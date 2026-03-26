/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/log.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_icm40609.h"

#if defined(USE_IMU_ICM40609)

#define ICM40609_RA_PWR_MGMT0                       0x4E
#define ICM40609_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM40609_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM40609_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)

#define ICM40609_RA_REG_BANK_SEL                    0x76
#define ICM40609_BANK_SELECT0                       0x00
#define ICM40609_BANK_SELECT1                       0x01
#define ICM40609_BANK_SELECT2                       0x02

#define ICM40609_RA_GYRO_CONFIG0                    0x4F
#define ICM40609_RA_ACCEL_CONFIG0                   0x50
#define ICM40609_RA_GYRO_ACCEL_CONFIG0              0x52
#define ICM40609_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM40609_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)

#define ICM40609_RA_GYRO_DATA_X1                    0x25
#define ICM40609_RA_ACCEL_DATA_X1                   0x1F

#define ICM40609_RA_INT_CONFIG                      0x14
#define ICM40609_INT1_MODE_PULSED                   (0 << 2)
#define ICM40609_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM40609_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM40609_RA_INT_CONFIG0                     0x63
#define ICM40609_UI_DRDY_INT_CLEAR_ON_SBR           (0 << 4)

#define ICM40609_RA_INT_CONFIG1                     0x64
#define ICM40609_INT_ASYNC_RESET_BIT                4
#define ICM40609_INT_TDEASSERT_DISABLE_BIT          5
#define ICM40609_INT_TDEASSERT_DISABLED             (1 << ICM40609_INT_TDEASSERT_DISABLE_BIT)
#define ICM40609_INT_TPULSE_DURATION_BIT            6
#define ICM40609_INT_TPULSE_DURATION_8              (1 << ICM40609_INT_TPULSE_DURATION_BIT)

#define ICM40609_RA_INT_SOURCE0                     0x65
#define ICM40609_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

#define ICM40609_RA_INTF_CONFIG0                    0x4C
#define ICM40609_SENSOR_DATA_ENDIAN_BIG             (1 << 4)

#define ICM40609_RA_INTF_CONFIG1                    0x4D
#define ICM40609_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM40609_INTF_CONFIG1_AFSR_DISABLE          0x40

#define ICM40609_RA_GYRO_CONFIG_STATIC3             0x0C  // User bank 1
#define ICM40609_RA_GYRO_CONFIG_STATIC4             0x0D  // User bank 1
#define ICM40609_RA_GYRO_CONFIG_STATIC5             0x0E  // User bank 1
#define ICM40609_RA_ACCEL_CONFIG_STATIC2            0x03  // User bank 2
#define ICM40609_RA_ACCEL_CONFIG_STATIC3            0x04  // User bank 2
#define ICM40609_RA_ACCEL_CONFIG_STATIC4            0x05  // User bank 2

typedef struct aafConfig_s {
    uint16_t freq;
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

// ICM40609 AAF table from datasheet table 5.2 (used in Betaflight driver).
static const aafConfig_t aafLUT40609[] = {
    { 42, 1, 1, 15 }, { 84, 2, 4, 13 }, { 126, 3, 9, 12 }, { 170, 4, 16, 11 }, { 213, 5, 25, 10 },
    { 258, 6, 36, 10 }, { 303, 7, 49, 9 }, { 348, 8, 64, 9 }, { 394, 9, 81, 9 }, { 441, 10, 100, 8 },
    { 488, 11, 122, 8 }, { 536, 12, 144, 8 }, { 585, 13, 170, 8 }, { 634, 14, 196, 8 }, { 684, 15, 224, 7 },
    { 734, 16, 256, 7 }, { 785, 17, 288, 7 }, { 837, 18, 324, 7 }, { 890, 19, 360, 6 }, { 943, 20, 400, 6 },
    { 997, 21, 440, 6 }, { 1051, 22, 488, 6 }, { 1107, 23, 528, 6 }, { 1163, 24, 576, 6 }, { 1220, 25, 624, 6 },
    { 1277, 26, 680, 6 }, { 1336, 27, 736, 5 }, { 1395, 28, 784, 5 }, { 1454, 29, 848, 5 }, { 1515, 30, 896, 5 },
    { 1577, 31, 960, 5 }, { 1639, 32, 1024, 5 }, { 1702, 33, 1088, 5 }, { 1766, 34, 1152, 5 }, { 1830, 35, 1232, 5 },
    { 1896, 36, 1296, 5 }, { 1962, 37, 1376, 4 }, { 2029, 38, 1440, 4 }, { 2097, 39, 1536, 4 }, { 2166, 40, 1600, 4 },
    { 2235, 41, 1696, 4 }, { 2306, 42, 1760, 4 }, { 2377, 43, 1856, 4 }, { 2449, 44, 1952, 4 }, { 2522, 45, 2016, 4 },
    { 2596, 46, 2112, 4 }, { 2671, 47, 2208, 4 }, { 2746, 48, 2304, 4 }, { 2823, 49, 2400, 4 }, { 2900, 50, 2496, 4 },
    { 2978, 51, 2592, 4 }, { 3057, 52, 2720, 4 }, { 3137, 53, 2816, 4 }, { 3217, 54, 2944, 3 }, { 3299, 55, 3008, 3 },
    { 3381, 56, 3136, 3 }, { 3464, 57, 3264, 3 }, { 3548, 58, 3392, 3 }, { 3633, 59, 3456, 3 }, { 3718, 60, 3584, 3 },
    { 3805, 61, 3712, 3 }, { 3892, 62, 3840, 3 }, { 3979, 63, 3968, 3 }, { 0, 0, 0, 0 }
};

static const aafConfig_t *getGyroAafConfig(const uint16_t desiredLpf);

// Temporary bring-up helper:
// emit a deterministic SPI waveform before normal WHO_AM_I probing.
// Sequence is read-type traffic (MSB=1 in first byte) to avoid accidental writes.
static void icm40609SpiWaveformTest(const busDevice_t *dev)
{
    if (dev->busType != BUSTYPE_SPI) {
        return;
    }

    uint8_t rx[4];
    const uint8_t txPattern[4] = { 0xAA, 0x55, 0xCC, 0x33 };

    // Send several times so logic analyzer has enough windows to catch.
    for (int i = 0; i < 8; i++) {
        busTransfer(dev, rx, txPattern, sizeof(txPattern));
        delay(2);
    }
}

static void setUserBank(const busDevice_t *dev, const uint8_t userBank)
{
    busWrite(dev, ICM40609_RA_REG_BANK_SEL, userBank & 7);
}

static uint16_t getAafFreq(const uint8_t gyroLpf)
{
    switch (gyroLpf) {
        default:
        case GYRO_LPF_256HZ:
            return 256;
        case GYRO_LPF_188HZ:
            return 188;
        case GYRO_LPF_98HZ:
            return 98;
        case GYRO_LPF_42HZ:
            return 42;
        case GYRO_LPF_20HZ:
            return 20;
        case GYRO_LPF_10HZ:
            return 10;
        case GYRO_LPF_5HZ:
            return 5;
        case GYRO_LPF_NONE:
            return 0;
    }
}

static const aafConfig_t *getGyroAafConfig(const uint16_t desiredLpf)
{
    const uint16_t desiredFreq = getAafFreq(desiredLpf);
    const aafConfig_t *candidate = &aafLUT40609[0];
    uint16_t bestDelta = UINT16_MAX;

    for (unsigned i = 0; aafLUT40609[i].freq != 0; i++) {
        const uint16_t currentFreq = aafLUT40609[i].freq;
        const uint16_t delta = ABS((int)desiredFreq - (int)currentFreq);
        if (delta < bestDelta) {
            bestDelta = delta;
            candidate = &aafLUT40609[i];
        }
    }

    LOG_VERBOSE(GYRO, "ICM40609 AAF CONFIG { %d, %d } -> { %d }; delt: %d deltSqr: %d, shift: %d",
        desiredLpf, desiredFreq, candidate->freq, candidate->delt, candidate->deltSqr, candidate->bitshift);

    return candidate;
}

static void icm40609AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4; // 16g full scale
}

static bool icm40609AccRead(accDev_t *acc)
{
    uint8_t data[6];

    if (!busReadBuf(acc->busDev, ICM40609_RA_ACCEL_DATA_X1, data, 6)) {
        return false;
    }

    acc->ADCRaw[X] = (float)int16_val_big_endian(data, 0);
    acc->ADCRaw[Y] = (float)int16_val_big_endian(data, 1);
    acc->ADCRaw[Z] = (float)int16_val_big_endian(data, 2);

    return true;
}

bool icm40609AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_ICM40609, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    mpuContextData_t *ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x4069) {
        return false;
    }

    acc->initFn = icm40609AccInit;
    acc->readFn = icm40609AccRead;
    acc->accAlign = acc->busDev->param;

    return true;
}

static const gyroFilterAndRateConfig_t icm40609GyroConfigs[] = {
    /*                            DLPF  ODR */
    { GYRO_LPF_256HZ,   8000,   { 6,    3  } },
    { GYRO_LPF_256HZ,   4000,   { 5,    4  } },
    { GYRO_LPF_256HZ,   2000,   { 3,    5  } },
    { GYRO_LPF_256HZ,   1000,   { 1,    6  } },
    { GYRO_LPF_256HZ,    500,   { 0,   15  } },

    { GYRO_LPF_188HZ,   1000,   { 3,    6  } },
    { GYRO_LPF_188HZ,    500,   { 1,   15  } },

    { GYRO_LPF_98HZ,    1000,   { 4,    6  } },
    { GYRO_LPF_98HZ,     500,   { 2,   15  } },

    { GYRO_LPF_42HZ,    1000,   { 6,    6  } },
    { GYRO_LPF_42HZ,     500,   { 4,   15  } },

    { GYRO_LPF_20HZ,    1000,   { 7,    6  } },
    { GYRO_LPF_20HZ,     500,   { 6,   15  } },

    { GYRO_LPF_10HZ,    1000,   { 7,    6  } },
    { GYRO_LPF_10HZ,     500,   { 7,   15  } }
};

static void icm40609AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t *dev = gyro->busDev;
    const gyroFilterAndRateConfig_t *config = chooseGyroConfig(
        gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs,
        &icm40609GyroConfigs[0], ARRAYLEN(icm40609GyroConfigs));

    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    setUserBank(dev, ICM40609_BANK_SELECT0);
    busWrite(dev, ICM40609_RA_PWR_MGMT0, ICM40609_PWR_MGMT0_TEMP_DISABLE_OFF | ICM40609_PWR_MGMT0_ACCEL_MODE_LN | ICM40609_PWR_MGMT0_GYRO_MODE_LN);
    delay(15);

    uint8_t intfConfig0Value;
    busRead(dev, ICM40609_RA_INTF_CONFIG0, &intfConfig0Value);
    intfConfig0Value |= ICM40609_SENSOR_DATA_ENDIAN_BIG;
    busWrite(dev, ICM40609_RA_INTF_CONFIG0, intfConfig0Value);
    delay(1);

    // ODR and full-scale
    busWrite(dev, ICM40609_RA_GYRO_CONFIG0, (0x00 << 5) | (config->gyroConfigValues[1] & 0x0F));  // 2000dps
    delay(15);
    busWrite(dev, ICM40609_RA_ACCEL_CONFIG0, (0x01 << 5) | (config->gyroConfigValues[1] & 0x0F)); // 16g
    delay(15);

    // Low-latency UI filter bandwidth
    busWrite(dev, ICM40609_RA_GYRO_ACCEL_CONFIG0, ICM40609_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM40609_GYRO_UI_FILT_BW_LOW_LATENCY);
    delay(15);

    if (gyro->lpf != GYRO_LPF_NONE) {
        const aafConfig_t *aafConfig = getGyroAafConfig(gyro->lpf);

        setUserBank(dev, ICM40609_BANK_SELECT1);
        busWrite(dev, ICM40609_RA_GYRO_CONFIG_STATIC3, aafConfig->delt);
        busWrite(dev, ICM40609_RA_GYRO_CONFIG_STATIC4, aafConfig->deltSqr & 0xFF);
        busWrite(dev, ICM40609_RA_GYRO_CONFIG_STATIC5, (aafConfig->deltSqr >> 8) | (aafConfig->bitshift << 4));

        // Keep accelerometer AAF aligned to the same hardware LPF target.
        setUserBank(dev, ICM40609_BANK_SELECT2);
        busWrite(dev, ICM40609_RA_ACCEL_CONFIG_STATIC2, aafConfig->delt << 1);
        busWrite(dev, ICM40609_RA_ACCEL_CONFIG_STATIC3, aafConfig->deltSqr & 0xFF);
        busWrite(dev, ICM40609_RA_ACCEL_CONFIG_STATIC4, (aafConfig->deltSqr >> 8) | (aafConfig->bitshift << 4));
    }

    setUserBank(dev, ICM40609_BANK_SELECT0);
    busWrite(dev, ICM40609_RA_INT_CONFIG, ICM40609_INT1_MODE_PULSED | ICM40609_INT1_DRIVE_CIRCUIT_PP | ICM40609_INT1_POLARITY_ACTIVE_HIGH);
    delay(15);

    busWrite(dev, ICM40609_RA_INT_CONFIG0, ICM40609_UI_DRDY_INT_CLEAR_ON_SBR);
    delay(100);

    busWrite(dev, ICM40609_RA_INT_SOURCE0, ICM40609_UI_DRDY_INT1_EN_ENABLED);

    uint8_t intConfig1Value;
    busRead(dev, ICM40609_RA_INT_CONFIG1, &intConfig1Value);
    intConfig1Value &= ~(1 << ICM40609_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM40609_INT_TPULSE_DURATION_8 | ICM40609_INT_TDEASSERT_DISABLED);
    busWrite(dev, ICM40609_RA_INT_CONFIG1, intConfig1Value);
    delay(15);

    uint8_t intfConfig1Value;
    busRead(dev, ICM40609_RA_INTF_CONFIG1, &intfConfig1Value);
    intfConfig1Value &= ~ICM40609_INTF_CONFIG1_AFSR_MASK;
    intfConfig1Value |= ICM40609_INTF_CONFIG1_AFSR_DISABLE;
    busWrite(dev, ICM40609_RA_INTF_CONFIG1, intfConfig1Value);
    delay(15);

    busSetSpeed(dev, BUS_SPEED_FAST);
}

static bool icm40609DeviceDetect(busDevice_t *dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);
    busWrite(dev, ICM40609_RA_PWR_MGMT0, 0x00);
    icm40609SpiWaveformTest(dev);

    do {
        delay(150);
        // Use a raw SPI transaction here to make WHO_AM_I probing explicit:
        // TX: [0x75 | 0x80, 0xFF], RX: [dummy, chip_id].
        // This avoids ambiguity from generic bus read wrappers during bring-up.
        if (dev->busType == BUSTYPE_SPI) {
            uint8_t tx[2] = { (uint8_t)(MPU_RA_WHO_AM_I | 0x80), 0xFF };
            uint8_t rx[2] = { 0, 0 };
            busTransfer(dev, rx, tx, sizeof(tx));
            tmp = rx[1];
        } else {
            busRead(dev, MPU_RA_WHO_AM_I, &tmp);
        }
        if (tmp == ICM40609_WHO_AM_I_CONST) {
            return true;
        }
    } while (attemptsRemaining--);

    return false;
}

static bool icm40609GyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    if (!busReadBuf(gyro->busDev, ICM40609_RA_GYRO_DATA_X1, data, 6)) {
        return false;
    }

    gyro->gyroADCRaw[X] = (float)int16_val_big_endian(data, 0);
    gyro->gyroADCRaw[Y] = (float)int16_val_big_endian(data, 1);
    gyro->gyroADCRaw[Z] = (float)int16_val_big_endian(data, 2);

    return true;
}

bool icm40609GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_ICM40609, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!icm40609DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected ICM40609 gyro.
    mpuContextData_t *ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x4069;

    gyro->initFn = icm40609AccAndGyroInit;
    gyro->readFn = icm40609GyroRead;
    gyro->intStatusFn = gyroCheckDataReady;
    gyro->temperatureFn = NULL;
    gyro->scale = 1.0f / 16.4f;
    gyro->gyroAlign = gyro->busDev->param;

    return true;
}

#endif
