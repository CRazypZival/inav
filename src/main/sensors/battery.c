/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

#include "platform.h"

#include "build/debug.h"
#include "build/atomic.h"

#include "common/maths.h"
#include "common/filter.h"

#include "config/config_reset.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/adc.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"
#include "fc/settings.h"

#include "flight/imu.h"
#include "flight/mixer.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "config/feature.h"

#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "rx/rx.h"

#include "fc/rc_controls.h"

#include "io/beeper.h"

#if defined(USE_FAKE_BATT_SENSOR)
#include "sensors/battery_sensor_fake.h"
#endif

#define ADCVREF 3300                            // in mV (3300 = 3.3V)

#define VBATT_CELL_FULL_MAX_DIFF 10             // Max difference with cell max voltage for the battery to be considered full (10mV steps)
#define VBATT_PRESENT_THRESHOLD 100             // Minimum voltage to consider battery present
#define VBATT_STABLE_DELAY 40                   // Delay after connecting battery to begin monitoring
#define VBATT_HYSTERESIS 10                     // Batt Hysteresis of +/-100mV for changing battery state
#define VBATT_LPF_FREQ  1                       // Battery voltage filtering cutoff
#define AMPERAGE_LPF_FREQ  1                    // Battery current filtering cutoff
#ifndef CURRENT_METER_LPF_FREQ
#define CURRENT_METER_LPF_FREQ AMPERAGE_LPF_FREQ
#endif
#ifdef CURRENT_METER_DUAL_CALIBRATION
#ifndef CURRENT_METER_DUAL_TRANSITION_UV_START
#define CURRENT_METER_DUAL_TRANSITION_UV_START 120000  // ADC pin voltage in uV
#endif
#ifndef CURRENT_METER_DUAL_TRANSITION_UV_END
#define CURRENT_METER_DUAL_TRANSITION_UV_END   200000  // ADC pin voltage in uV
#endif
#ifndef CURRENT_METER_DUAL_LOW_USE_CLI
#define CURRENT_METER_DUAL_LOW_USE_CLI 1               // 1: low-range uses CLI/runtime values
#endif
#ifndef CURRENT_METER_LOW_SCALE
#define CURRENT_METER_LOW_SCALE CURRENT_METER_SCALE
#endif
#ifndef CURRENT_METER_LOW_OFFSET
#define CURRENT_METER_LOW_OFFSET CURRENT_METER_OFFSET
#endif
#endif
#define IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH 10 // Minimum sample count to consider calculated power supply impedance as stable


// Battery monitoring stuff
static uint8_t batteryCellCount;                // cell count
static uint16_t batteryFullVoltage;
static uint16_t batteryWarningVoltage;
static uint16_t batteryCriticalVoltage;
static uint32_t batteryRemainingCapacity = 0;
static bool batteryUseCapacityThresholds = false;
static bool batteryFullWhenPluggedIn = false;
static bool profileAutoswitchDisable = false;

static uint16_t vbat = 0;                       // battery voltage in 0.01V steps (filtered)
static uint16_t powerSupplyImpedance = 0;       // calculated impedance in milliohm
static uint16_t sagCompensatedVBat = 0;         // calculated no load vbat
static bool powerSupplyImpedanceIsValid = false;

static int16_t amperage = 0;                    // amperage read by current sensor in centiampere (1/100th A)
static int32_t power = 0;                       // power draw in cW (0.01W resolution)
static int32_t mAhDrawn = 0;                    // milliampere hours drawn from the battery since start
static int32_t mWhDrawn = 0;                    // energy (milliWatt hours) drawn from the battery since start

static uint16_t currentMeterLastAdcRaw = 0;
static int16_t currentMeterLastScale = 0;
static int16_t currentMeterLastOffset = 0;

#define CURRENT_METER_RECORD_STEP_A 10
#define CURRENT_METER_RECORD_MAX_BUCKETS 101    // 0A..1000A, 10A bins
typedef struct currentMeterRecordBucket_s {
    uint32_t adcRawSum;
    uint16_t sampleCount;
    int16_t scale;
    int16_t offset;
    bool valid;
} currentMeterRecordBucket_t;

static currentMeterRecordBucket_t currentMeterRecord[CURRENT_METER_RECORD_MAX_BUCKETS];
static uint8_t currentMeterRecordMaxBucket = 0;

batteryState_e batteryState;
const batteryProfile_t *currentBatteryProfile;

PG_REGISTER_ARRAY_WITH_RESET_FN(batteryProfile_t, MAX_BATTERY_PROFILE_COUNT, batteryProfiles, PG_BATTERY_PROFILES, 2);

void pgResetFn_batteryProfiles(batteryProfile_t *instance)
{
    for (int i = 0; i < MAX_BATTERY_PROFILE_COUNT; i++) {
        RESET_CONFIG(batteryProfile_t, &instance[i],
#ifdef USE_ADC
            .cells = SETTING_BAT_CELLS_DEFAULT,

            .voltage = {
                .cellDetect = SETTING_VBAT_CELL_DETECT_VOLTAGE_DEFAULT,
                .cellMax = SETTING_VBAT_MAX_CELL_VOLTAGE_DEFAULT,
                .cellMin = SETTING_VBAT_MIN_CELL_VOLTAGE_DEFAULT,
                .cellWarning = SETTING_VBAT_WARNING_CELL_VOLTAGE_DEFAULT
            },
#endif

            .capacity = {
                .value = SETTING_BATTERY_CAPACITY_DEFAULT,
                .warning = SETTING_BATTERY_CAPACITY_WARNING_DEFAULT,
                .critical = SETTING_BATTERY_CAPACITY_CRITICAL_DEFAULT,
                .unit = SETTING_BATTERY_CAPACITY_UNIT_DEFAULT,
            },

            .controlRateProfile = 0,

            .motor = {
                .throttleIdle = SETTING_THROTTLE_IDLE_DEFAULT,
                .throttleScale = SETTING_THROTTLE_SCALE_DEFAULT,
#ifdef USE_DSHOT
                .turtleModePowerFactor = SETTING_TURTLE_MODE_POWER_FACTOR_DEFAULT,
#endif
            },

            .failsafe_throttle = SETTING_FAILSAFE_THROTTLE_DEFAULT,                                 // default throttle off.

            .nav = {
                .mc = {
                    .hover_throttle = SETTING_NAV_MC_HOVER_THR_DEFAULT,
                },

                .fw = {
                    .cruise_throttle = SETTING_NAV_FW_CRUISE_THR_DEFAULT,
                    .max_throttle = SETTING_NAV_FW_MAX_THR_DEFAULT,
                    .min_throttle = SETTING_NAV_FW_MIN_THR_DEFAULT,
                    .pitch_to_throttle = SETTING_NAV_FW_PITCH2THR_DEFAULT,                          // pwm units per degree of pitch (10pwm units ~ 1% throttle)
                    .launch_throttle = SETTING_NAV_FW_LAUNCH_THR_DEFAULT,
                    .launch_idle_throttle = SETTING_NAV_FW_LAUNCH_IDLE_THR_DEFAULT,                 // Motor idle or MOTOR_STOP
                }
            },

#if defined(USE_POWER_LIMITS)
            .powerLimits = {
                .continuousCurrent = SETTING_LIMIT_CONT_CURRENT_DEFAULT,                            // dA
                .burstCurrent = SETTING_LIMIT_BURST_CURRENT_DEFAULT,                                // dA
                .burstCurrentTime = SETTING_LIMIT_BURST_CURRENT_TIME_DEFAULT,                       // dS
                .burstCurrentFalldownTime = SETTING_LIMIT_BURST_CURRENT_FALLDOWN_TIME_DEFAULT,      // dS
#ifdef USE_ADC
                .continuousPower = SETTING_LIMIT_CONT_POWER_DEFAULT,                                // dW
                .burstPower = SETTING_LIMIT_BURST_POWER_DEFAULT,                                    // dW
                .burstPowerTime = SETTING_LIMIT_BURST_POWER_TIME_DEFAULT,                           // dS
                .burstPowerFalldownTime = SETTING_LIMIT_BURST_POWER_FALLDOWN_TIME_DEFAULT,          // dS
#endif // USE_ADC
            }
#endif // USE_POWER_LIMITS

        );
    }
}

PG_REGISTER_WITH_RESET_TEMPLATE(batteryMetersConfig_t, batteryMetersConfig, PG_BATTERY_METERS_CONFIG, 2);

PG_RESET_TEMPLATE(batteryMetersConfig_t, batteryMetersConfig,

#ifdef USE_ADC
    .voltage = {
        .type = SETTING_VBAT_METER_TYPE_DEFAULT,
        .scale = VBAT_SCALE_DEFAULT,
    },
#endif

    .current = {
        .type = SETTING_CURRENT_METER_TYPE_DEFAULT,
        .scale = CURRENT_METER_SCALE,
        .offset = CURRENT_METER_OFFSET
    },

    .currentDual = {
        .enabled = CURRENT_METER_DUAL_CALIBRATION_DEFAULT,
        .lowUseCli = CURRENT_METER_DUAL_LOW_USE_CLI_DEFAULT,
        .lowScale = CURRENT_METER_LOW_SCALE_DEFAULT,
        .lowOffset = CURRENT_METER_LOW_OFFSET_DEFAULT,
        .transitionUvStart = CURRENT_METER_DUAL_TRANSITION_UV_START_DEFAULT,
        .transitionUvEnd = CURRENT_METER_DUAL_TRANSITION_UV_END_DEFAULT,
        .highScale = CURRENT_METER_HIGH_SCALE_DEFAULT,
        .highOffset = CURRENT_METER_HIGH_OFFSET_DEFAULT,
    },

    .voltageSource = SETTING_BAT_VOLTAGE_SRC_DEFAULT,

    .cruise_power = SETTING_CRUISE_POWER_DEFAULT,
    .idle_power = SETTING_IDLE_POWER_DEFAULT,
    .rth_energy_margin = SETTING_RTH_ENERGY_MARGIN_DEFAULT,

    .throttle_compensation_weight = SETTING_THR_COMP_WEIGHT_DEFAULT

);

void batteryInit(void)
{
    batteryState = BATTERY_NOT_PRESENT;
    batteryCellCount = 0;
    batteryFullVoltage = 0;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
}

#ifdef USE_ADC
// profileDetect() profile sorting compare function
static int profile_compare(profile_comp_t *a, profile_comp_t *b) {
    if (a->max_voltage < b->max_voltage)
        return -1;
    else if (a->max_voltage > b->max_voltage)
        return 1;
    else
        return 0;
}

// Find profile matching plugged battery for profile_autoselect
static int8_t profileDetect(void) {
    profile_comp_t profile_comp_array[MAX_BATTERY_PROFILE_COUNT];

    // Prepare profile sort
    for (uint8_t i = 0; i < MAX_BATTERY_PROFILE_COUNT; ++i) {
        const batteryProfile_t *profile = batteryProfiles(i);
        profile_comp_array[i].profile_index = i;
        profile_comp_array[i].max_voltage = profile->cells * profile->voltage.cellDetect;
    }

    // Sort profiles by max voltage
    qsort(profile_comp_array, MAX_BATTERY_PROFILE_COUNT, sizeof(*profile_comp_array), (int (*)(const void *, const void *))profile_compare);

    // Return index of the first profile where vbat <= profile_max_voltage
    for (uint8_t i = 0; i < MAX_BATTERY_PROFILE_COUNT; ++i)
        if ((profile_comp_array[i].max_voltage > 0) && (vbat <= profile_comp_array[i].max_voltage))
            return profile_comp_array[i].profile_index;

    // No matching profile found
    return -1;
}
#endif

void setBatteryProfile(uint8_t profileIndex)
{
    if (profileIndex >= MAX_BATTERY_PROFILE_COUNT) {
        profileIndex = 0;
    }
    currentBatteryProfile = batteryProfiles(profileIndex);
    if ((currentBatteryProfile->controlRateProfile > 0) && (currentBatteryProfile->controlRateProfile < MAX_CONTROL_RATE_PROFILE_COUNT)) {
        setConfigProfile(currentBatteryProfile->controlRateProfile - 1);
    }
}

void activateBatteryProfile(void)
{
    static int8_t previous_battery_profile_index = -1;
    // Don't call batteryInit if the battery profile was not changed to prevent batteryCellCount to be reset while adjusting board alignment
    // causing the beeper to be silent when it is disabled while the board is connected through USB (beeper -ON_USB)
    if (systemConfig()->current_battery_profile_index != previous_battery_profile_index) {
        batteryInit();
        previous_battery_profile_index = systemConfig()->current_battery_profile_index;
    }
}

#ifdef USE_ADC
static void updateBatteryVoltage(timeUs_t timeDelta, bool justConnected)
{
    static pt1Filter_t vbatFilterState;

    switch (batteryMetersConfig()->voltage.type) {
        case VOLTAGE_SENSOR_ADC:
            {
                vbat = getVBatSample();
                break;
            }
#if defined(USE_ESC_SENSOR)
        case VOLTAGE_SENSOR_ESC:
            {
                escSensorData_t * escSensor = escSensorGetData();
                if (escSensor && escSensor->dataAge <= ESC_DATA_MAX_AGE) {
                    vbat = escSensor->voltage;
                }
                else {
                    vbat = 0;
                }
            }
            break;
#endif
        
#if defined(USE_FAKE_BATT_SENSOR)
    case VOLTAGE_SENSOR_FAKE:
        vbat = fakeBattSensorGetVBat();
        break;
#endif
    case VOLTAGE_SENSOR_NONE:
        default:
            vbat = 0;
            break;
    }

#ifdef USE_SIMULATOR
    if (ARMING_FLAG(SIMULATOR_MODE_HITL) && SIMULATOR_HAS_OPTION(HITL_SIMULATE_BATTERY)) {
        vbat = ((uint16_t)simulatorData.vbat)*10;
        return;
    }
#endif

    if (justConnected) {
        pt1FilterReset(&vbatFilterState, vbat);
    } else {
        vbat = pt1FilterApply4(&vbatFilterState, vbat, VBATT_LPF_FREQ, US2S(timeDelta));
    }
}

batteryState_e checkBatteryVoltageState(void)
{
    uint16_t stateVoltage = getBatteryVoltage();
    switch (batteryState)
    {
        case BATTERY_OK:
            if (stateVoltage <= (batteryWarningVoltage - VBATT_HYSTERESIS)) {
                return BATTERY_WARNING;
            }
            break;
        case BATTERY_WARNING:
            if (stateVoltage <= (batteryCriticalVoltage - VBATT_HYSTERESIS)) {
                return BATTERY_CRITICAL;
            } else if (stateVoltage > (batteryWarningVoltage + VBATT_HYSTERESIS)){
                return BATTERY_OK;
            }
            break;
        case BATTERY_CRITICAL:
            if (stateVoltage > (batteryCriticalVoltage + VBATT_HYSTERESIS)) {
                return BATTERY_WARNING;
            }
            break;
        default:
            break;
    }

    return batteryState;
}

static void checkBatteryCapacityState(void)
{
    if (batteryRemainingCapacity == 0) {
        batteryState = BATTERY_CRITICAL;
    } else if (batteryRemainingCapacity <= currentBatteryProfile->capacity.warning - currentBatteryProfile->capacity.critical) {
        batteryState = BATTERY_WARNING;
    }
}

void batteryUpdate(timeUs_t timeDelta)
{
    /* battery has just been connected*/
    if (batteryState == BATTERY_NOT_PRESENT && vbat > VBATT_PRESENT_THRESHOLD) {

        /* Actual battery state is calculated below, this is really BATTERY_PRESENT */
        batteryState = BATTERY_OK;
        /* wait for VBatt to stabilise then we can calc number of cells
        (using the filtered value takes a long time to ramp up)
        We only do this on the ground so don't care if we do block, not
        worse than original code anyway*/
        delay(VBATT_STABLE_DELAY);
        updateBatteryVoltage(timeDelta, true);

        int8_t detectedProfileIndex = -1;
        if (feature(FEATURE_BAT_PROFILE_AUTOSWITCH) && (!profileAutoswitchDisable))
            detectedProfileIndex = profileDetect();

        if (detectedProfileIndex != -1) {
            systemConfigMutable()->current_battery_profile_index = detectedProfileIndex;
            setBatteryProfile(detectedProfileIndex);
            batteryCellCount = currentBatteryProfile->cells;
        } else if (currentBatteryProfile->cells > 0)
            batteryCellCount = currentBatteryProfile->cells;
        else {
            batteryCellCount = (vbat / currentBatteryProfile->voltage.cellDetect) + 1;
            // Assume there are no 7S, 9S and 11S batteries so round up to 8S, 10S and 12S respectively
            if (batteryCellCount == 7 || batteryCellCount == 9 || batteryCellCount == 11) {
                batteryCellCount += 1;
            }
            batteryCellCount = MIN(batteryCellCount, 12);
        }

        batteryFullVoltage = batteryCellCount * currentBatteryProfile->voltage.cellMax;
        batteryWarningVoltage = batteryCellCount * currentBatteryProfile->voltage.cellWarning;
        batteryCriticalVoltage = batteryCellCount * currentBatteryProfile->voltage.cellMin;

        batteryFullWhenPluggedIn = vbat >= (batteryFullVoltage - batteryCellCount * VBATT_CELL_FULL_MAX_DIFF);
        batteryUseCapacityThresholds = isAmperageConfigured() && batteryFullWhenPluggedIn && (currentBatteryProfile->capacity.value > 0) &&
                                           (currentBatteryProfile->capacity.warning > 0) && (currentBatteryProfile->capacity.critical > 0);

    } else {
        updateBatteryVoltage(timeDelta, false);

        /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of VBATT_PRESENT_THRESHOLD */
        if (batteryState != BATTERY_NOT_PRESENT && vbat <= VBATT_PRESENT_THRESHOLD) {
            batteryState = BATTERY_NOT_PRESENT;
            batteryCellCount = 0;
            batteryWarningVoltage = 0;
            batteryCriticalVoltage = 0;
        }
    }

    if (batteryState != BATTERY_NOT_PRESENT) {

        if ((currentBatteryProfile->capacity.value > 0) && batteryFullWhenPluggedIn) {
            uint32_t capacityDiffBetweenFullAndEmpty = currentBatteryProfile->capacity.value - currentBatteryProfile->capacity.critical;
            int32_t drawn = (currentBatteryProfile->capacity.unit == BAT_CAPACITY_UNIT_MWH ? mWhDrawn : mAhDrawn);
            batteryRemainingCapacity = (drawn > (int32_t)capacityDiffBetweenFullAndEmpty ? 0 : capacityDiffBetweenFullAndEmpty - drawn);
        }

        if (batteryUseCapacityThresholds) {
            checkBatteryCapacityState();
        } else {
            batteryState = checkBatteryVoltageState();
        }

        // handle beeper
        if (ARMING_FLAG(ARMED) || !ARMING_FLAG(WAS_EVER_ARMED) || failsafeIsActive())
            switch (batteryState) {
                case BATTERY_WARNING:
                    beeper(BEEPER_BAT_LOW);
                    break;
                case BATTERY_CRITICAL:
                    beeper(BEEPER_BAT_CRIT_LOW);
                    break;
                default:
                    break;
            }
    }
}
#endif

batteryState_e getBatteryState(void)
{
    return batteryState;
}

bool batteryWasFullWhenPluggedIn(void)
{
    return batteryFullWhenPluggedIn;
}

bool batteryUsesCapacityThresholds(void)
{
    return batteryUseCapacityThresholds;
}

bool isBatteryVoltageConfigured(void)
{
    return feature(FEATURE_VBAT);
}

#ifdef USE_ADC
uint16_t getVBatSample(void) {
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.01V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 1100 = 11:1 voltage divider (10k:1k)
    return (uint64_t)adcGetChannel(ADC_BATTERY) * batteryMetersConfig()->voltage.scale * ADCVREF / (0xFFF * 1000);
}
#endif

uint16_t getBatteryVoltage(void)
{
    if (batteryMetersConfig()->voltageSource == BAT_VOLTAGE_SAG_COMP) {
        return sagCompensatedVBat;
    }

    return vbat;
}

uint16_t getBatteryRawVoltage(void)
{
    return vbat;
}

uint16_t getBatterySagCompensatedVoltage(void)
{
    return sagCompensatedVBat;
}

float calculateThrottleCompensationFactor(void)
{
    return 1.0f + ((float)batteryFullVoltage / sagCompensatedVBat - 1.0f) * batteryMetersConfig()->throttle_compensation_weight;
}

uint8_t getBatteryCellCount(void)
{
    return batteryCellCount;
}

uint16_t getBatteryAverageCellVoltage(void)
{
    if (batteryCellCount > 0) {
        return getBatteryVoltage() / batteryCellCount;
    }
    return 0;
}

uint16_t getBatteryRawAverageCellVoltage(void)
{
    if (batteryCellCount > 0) {
        return vbat / batteryCellCount;
    }
    return 0;
}

uint16_t getBatterySagCompensatedAverageCellVoltage(void)
{
    if (batteryCellCount > 0) {
        return sagCompensatedVBat / batteryCellCount;
    }
    return 0;
}

uint32_t getBatteryRemainingCapacity(void)
{
    return batteryRemainingCapacity;
}

bool isAmperageConfigured(void)
{
    return feature(FEATURE_CURRENT_METER) && batteryMetersConfig()->current.type != CURRENT_SENSOR_NONE;
}

int16_t getAmperage(void)
{
    return amperage;
}

static int16_t getAmperageSampleInternal(timeUs_t timeDelta, bool useFilteredTransitionMv)
{
    static pt1Filter_t transitionMvFilterState;
    static bool transitionMvFilterInitialized = false;

    const uint16_t adcRaw = adcGetChannel(ADC_CURRENT);
    const int32_t microvolts = ((uint32_t)adcRaw * ADCVREF * 100) / 0xFFF * 10;
    const int32_t adcPinVoltageUv = ((uint32_t)adcRaw * ADCVREF * 1000) / 0xFFF;
    const uint16_t adcPinVoltageMv = (uint16_t)(adcPinVoltageUv / 1000);
    const batteryMetersConfig_t *cfg = batteryMetersConfig();
    int16_t lowScale = cfg->current.scale;
    int16_t lowOffset = cfg->current.offset;

    if (cfg->currentDual.enabled && !cfg->currentDual.lowUseCli) {
        lowScale = cfg->currentDual.lowScale;
        lowOffset = cfg->currentDual.lowOffset;
    }

    if (!lowScale) {
        return 0;
    }

    const int16_t lowAmperageSample = (microvolts - (int32_t)lowOffset * 100) / lowScale;
    currentMeterLastAdcRaw = adcRaw;
    currentMeterLastScale = lowScale;
    currentMeterLastOffset = lowOffset;

    if (!cfg->currentDual.enabled || !cfg->currentDual.highScale) {
        return lowAmperageSample;
    }

    const int16_t highAmperageSample = (microvolts - (int32_t)cfg->currentDual.highOffset * 100) / cfg->currentDual.highScale;
    uint32_t transitionVoltageUv = adcPinVoltageUv;

    if (useFilteredTransitionMv) {
        if (!transitionMvFilterInitialized) {
            pt1FilterReset(&transitionMvFilterState, adcPinVoltageMv);
            transitionMvFilterInitialized = true;
        } else {
            transitionVoltageUv = (uint32_t)(pt1FilterApply4(&transitionMvFilterState, adcPinVoltageMv, CURRENT_METER_LPF_FREQ, US2S(timeDelta)) * 1000.0f);
        }
    }

    if (transitionVoltageUv <= cfg->currentDual.transitionUvStart) {
        return lowAmperageSample;
    }

    if (transitionVoltageUv >= cfg->currentDual.transitionUvEnd) {
        currentMeterLastScale = cfg->currentDual.highScale;
        currentMeterLastOffset = cfg->currentDual.highOffset;
        return highAmperageSample;
    }

    if (cfg->currentDual.transitionUvEnd > cfg->currentDual.transitionUvStart) {
        // Linear blend in transition zone to avoid abrupt jumps.
        const int32_t blendNum = transitionVoltageUv - cfg->currentDual.transitionUvStart;
        const int32_t blendDen = cfg->currentDual.transitionUvEnd - cfg->currentDual.transitionUvStart;
        currentMeterLastScale = (int16_t)(((int32_t)lowScale * (blendDen - blendNum) + (int32_t)cfg->currentDual.highScale * blendNum) / blendDen);
        currentMeterLastOffset = (int16_t)(((int32_t)lowOffset * (blendDen - blendNum) + (int32_t)cfg->currentDual.highOffset * blendNum) / blendDen);
        return (int16_t)(((int32_t)lowAmperageSample * (blendDen - blendNum) + (int32_t)highAmperageSample * blendNum) / blendDen);
    }

    return lowAmperageSample;
}

int16_t getAmperageSample(void)
{
    return getAmperageSampleInternal(0, false);
}

int32_t getPower(void)
{
    return power;
}

int32_t getMAhDrawn(void)
{
    return mAhDrawn;
}

int32_t getMWhDrawn(void)
{
    return mWhDrawn;
}

void currentMeterUpdate(timeUs_t timeDelta)
{
    static pt1Filter_t amperageFilterState;
    static int64_t mAhdrawnRaw = 0;
    int16_t recordAmperage = amperage;

    switch (batteryMetersConfig()->current.type) {
        case CURRENT_SENSOR_ADC:
            {
                const int16_t rawAmperageSample = getAmperageSampleInternal(timeDelta, true);
                recordAmperage = rawAmperageSample;
                amperage = pt1FilterApply4(&amperageFilterState, rawAmperageSample, AMPERAGE_LPF_FREQ, US2S(timeDelta));
                break;
            }
        case CURRENT_SENSOR_VIRTUAL:
            amperage = batteryMetersConfig()->current.offset;
            if (ARMING_FLAG(ARMED)) {
                navigationFSMStateFlags_t stateFlags = navGetCurrentStateFlags();
                bool allNav = navConfig()->general.flags.nav_overrides_motor_stop == NOMS_ALL_NAV && posControl.navState != NAV_STATE_IDLE;
                bool autoNav = navConfig()->general.flags.nav_overrides_motor_stop == NOMS_AUTO_ONLY && (stateFlags & (NAV_AUTO_RTH | NAV_AUTO_WP));
                int32_t throttleOffset;

                if (allNav || autoNav) {    // account for motors running in Nav modes with throttle low + motor stop
                    throttleOffset = (int32_t)rcCommand[THROTTLE] - 1000;
                } else {
                    throttleOffset = (throttleStickIsLow() && ifMotorstopFeatureEnabled()) ? 0 : (int32_t)rcCommand[THROTTLE] - 1000;
                }
                int32_t throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50);
                amperage += throttleFactor * batteryMetersConfig()->current.scale / 1000;
            }
            break;
#if defined(USE_ESC_SENSOR)
        case CURRENT_SENSOR_ESC:
            {
                escSensorData_t * escSensor = escSensorGetData();
                if (escSensor && escSensor->dataAge <= ESC_DATA_MAX_AGE) {
                    amperage = pt1FilterApply4(&amperageFilterState, escSensor->current, AMPERAGE_LPF_FREQ, US2S(timeDelta));
                }
                else {
                    amperage = 0;
                }
            }
            break;
#endif

#if defined(USE_FAKE_BATT_SENSOR)
        case CURRENT_SENSOR_FAKE:
            amperage = fakeBattSensorGetAmerperage();
            break;
#endif
        case CURRENT_SENSOR_NONE:
        default:
            amperage = 0;
            break;
    }

    // Clamp amperage to positive values
    amperage = MAX(0, amperage);
    recordAmperage = MAX(0, recordAmperage);

    if (batteryMetersConfig()->current.type == CURRENT_SENSOR_ADC && currentMeterLastScale != 0) {
        const uint16_t bucketRaw = (uint16_t)(recordAmperage / (CURRENT_METER_RECORD_STEP_A * 100)); // 10A bins, amperage in cA
        const uint8_t bucketIndex = (uint8_t)MIN(bucketRaw, (uint16_t)(CURRENT_METER_RECORD_MAX_BUCKETS - 1));
        currentMeterRecordBucket_t *bucket = &currentMeterRecord[bucketIndex];

        if (bucket->valid && (bucket->scale != currentMeterLastScale || bucket->offset != currentMeterLastOffset)) {
            // Calibration regime changed within this bin; restart bin stats to keep report coherent.
            bucket->adcRawSum = 0;
            bucket->sampleCount = 0;
        }

        bucket->adcRawSum += currentMeterLastAdcRaw;
        if (bucket->sampleCount < UINT16_MAX) {
            bucket->sampleCount++;
        }
        bucket->scale = currentMeterLastScale;
        bucket->offset = currentMeterLastOffset;
        bucket->valid = true;
        if (bucketIndex > currentMeterRecordMaxBucket) {
            currentMeterRecordMaxBucket = bucketIndex;
        }
    }

    // Work around int64 math compiler bug, don't change it unless the bug has been fixed !
    // should be: mAhdrawnRaw += (int64_t)amperage * timeDelta / 1000;
    mAhdrawnRaw += (int64_t)((int32_t)amperage * timeDelta) / 1000;

    mAhDrawn = mAhdrawnRaw / (3600 * 100);
}

void currentMeterRecordReset(void)
{
    for (int i = 0; i < CURRENT_METER_RECORD_MAX_BUCKETS; i++) {
        currentMeterRecord[i].adcRawSum = 0;
        currentMeterRecord[i].sampleCount = 0;
        currentMeterRecord[i].scale = 0;
        currentMeterRecord[i].offset = 0;
        currentMeterRecord[i].valid = false;
    }
    currentMeterRecordMaxBucket = 0;
}

uint8_t currentMeterRecordGetMaxBucket(void)
{
    return currentMeterRecordMaxBucket;
}

bool currentMeterRecordGetBucket(uint8_t bucketIndex, uint16_t *adcRaw, uint16_t *adcMv, int16_t *scale, int16_t *offset)
{
    if (bucketIndex >= CURRENT_METER_RECORD_MAX_BUCKETS) {
        return false;
    }

    uint32_t adcRawSum = 0;
    uint16_t sampleCount = 0;
    int16_t bucketScale = 0;
    int16_t bucketOffset = 0;
    bool bucketValid = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        const currentMeterRecordBucket_t *bucket = &currentMeterRecord[bucketIndex];
        adcRawSum = bucket->adcRawSum;
        sampleCount = bucket->sampleCount;
        bucketScale = bucket->scale;
        bucketOffset = bucket->offset;
        bucketValid = bucket->valid;
    }

    if (!bucketValid || sampleCount == 0) {
        return false;
    }

    const uint16_t rawAverage = (uint16_t)(adcRawSum / sampleCount);

    if (adcRaw) {
        *adcRaw = rawAverage;
    }
    if (adcMv) {
        *adcMv = (uint16_t)(((uint32_t)rawAverage * ADCVREF) / 0xFFF);
    }
    if (scale) {
        *scale = bucketScale;
    }
    if (offset) {
        *offset = bucketOffset;
    }

    return true;
}

void powerMeterUpdate(timeUs_t timeDelta)
{
    static int64_t mWhDrawnRaw = 0;
    power = (int32_t)amperage * vbat / 100; // power unit is cW (0.01W resolution)
    int32_t heatLossesCompensatedPower_mW = (int32_t)amperage * vbat / 10 + sq((int64_t)amperage) * powerSupplyImpedance / 10000;

    // Work around int64 math compiler bug, don't change it unless the bug has been fixed !
    // should be: mWhDrawnRaw += (int64_t)heatLossesCompensatedPower_mW * timeDelta / 10000;
    mWhDrawnRaw += (int64_t)((int64_t)heatLossesCompensatedPower_mW * timeDelta) / 10000;

    mWhDrawn = mWhDrawnRaw / (3600 * 100);
}

// calculate true power including heat losses in power supply (battery + wires)
// power is in cW (0.01W)
// batteryWarningVoltage is in cV (0.01V)
int32_t heatLossesCompensatedPower(int32_t power)
{
    return power + sq(power * 100 / batteryWarningVoltage) * powerSupplyImpedance / 100000;
}

void sagCompensatedVBatUpdate(timeUs_t currentTime, timeUs_t timeDelta)
{
    static timeUs_t recordTimestamp = 0;
    static int16_t amperageRecord;
    static uint16_t vbatRecord;
    static uint8_t impedanceSampleCount = 0;
    static pt1Filter_t impedanceFilterState;
    static pt1Filter_t sagCompVBatFilterState;
    static batteryState_e last_battery_state = BATTERY_NOT_PRESENT;

    if ((batteryState != BATTERY_NOT_PRESENT) && (last_battery_state == BATTERY_NOT_PRESENT)) {
        pt1FilterReset(&sagCompVBatFilterState, vbat);
        pt1FilterReset(&impedanceFilterState, 0);
    }

    if (batteryState == BATTERY_NOT_PRESENT) {

        recordTimestamp = 0;
        impedanceSampleCount = 0;
        powerSupplyImpedance = 0;
        powerSupplyImpedanceIsValid = false;
        sagCompensatedVBat = vbat;

    } else {

        if (cmpTimeUs(currentTime, recordTimestamp) > MS2US(500))
            recordTimestamp = 0;

        if (!recordTimestamp) {
            amperageRecord = amperage;
            vbatRecord = vbat;
            recordTimestamp = currentTime;
        } else if ((amperage - amperageRecord >= 200) && ((int16_t)vbatRecord - vbat >= 4)) {

            uint16_t impedanceSample = (int32_t)(vbatRecord - vbat) * 1000 / (amperage - amperageRecord);

            if (impedanceSampleCount <= IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH) {
                impedanceSampleCount += 1;
            }

            if (impedanceFilterState.state) {
                pt1FilterSetTimeConstant(&impedanceFilterState, impedanceSampleCount > IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH ? 1.2 : 0.5);
                pt1FilterApply3(&impedanceFilterState, impedanceSample, US2S(timeDelta));
            } else {
                pt1FilterReset(&impedanceFilterState, impedanceSample);
            }

            if (impedanceSampleCount > IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH) {
                powerSupplyImpedance = lrintf(pt1FilterGetLastOutput(&impedanceFilterState));
                powerSupplyImpedanceIsValid = true;
            }

        }

        uint16_t sagCompensatedVBatSample = MIN(batteryFullVoltage, vbat + (int32_t)powerSupplyImpedance * amperage / 1000);
        pt1FilterSetTimeConstant(&sagCompVBatFilterState, sagCompensatedVBatSample < pt1FilterGetLastOutput(&sagCompVBatFilterState) ? 40 : 500);
        sagCompensatedVBat = lrintf(pt1FilterApply3(&sagCompVBatFilterState, sagCompensatedVBatSample, US2S(timeDelta)));
    }

    DEBUG_SET(DEBUG_SAG_COMP_VOLTAGE, 0, powerSupplyImpedance);
    DEBUG_SET(DEBUG_SAG_COMP_VOLTAGE, 1, sagCompensatedVBat);

    last_battery_state = batteryState;
}

uint8_t calculateBatteryPercentage(void)
{
    if (batteryState == BATTERY_NOT_PRESENT)
        return 0;

    if (batteryFullWhenPluggedIn && isAmperageConfigured() && (currentBatteryProfile->capacity.value > 0) && (currentBatteryProfile->capacity.critical > 0)) {
        uint32_t capacityDiffBetweenFullAndEmpty = currentBatteryProfile->capacity.value - currentBatteryProfile->capacity.critical;
        return constrain(batteryRemainingCapacity * 100 / capacityDiffBetweenFullAndEmpty, 0, 100);
    } else
        return constrain((getBatteryVoltage() - batteryCriticalVoltage) * 100L / (batteryFullVoltage - batteryCriticalVoltage), 0, 100);
}

void batteryDisableProfileAutoswitch(void) {
    profileAutoswitchDisable = true;
}

bool isPowerSupplyImpedanceValid(void) {
    return powerSupplyImpedanceIsValid;
}

uint16_t getPowerSupplyImpedance(void) {
    return powerSupplyImpedance;
}

// returns cW (0.01W)
int32_t calculateAveragePower(void) {
    return (int64_t)mWhDrawn * 360 / getFlightTime();
}

// returns mWh / meter
int32_t calculateAverageEfficiency(void) {
    return getFlyingEnergy() * 100 / getTotalTravelDistance();
}
