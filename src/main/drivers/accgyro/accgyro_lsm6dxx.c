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
 * from atbetaflight https://github.com/flightng/atbetaflight
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_lsm6dxx.h"

#if defined(USE_IMU_LSM6DXX)

typedef struct __attribute__ ((__packed__)) lsm6DContextData_s {
    uint16_t    chipMagicNumber;
    uint8_t     lastReadStatus;
    uint8_t     __padding_dummy;
    uint8_t     accRaw[6];
    uint8_t     gyroRaw[6];
} lsm6DContextData_t;

#define LSM6DSO_CHIP_ID 0x6C
#define LSM6DSL_CHIP_ID 0x6A
#define LSM6DS3_CHIP_ID 0x69
#define LSM6DSV16X_CHIP_ID 0x70
#define LSM6DSV32X_CHIP_ID 0x65
#define LSM6DSK320X_CHIP_ID 0x75

static uint8_t lsm6dID = 0x6C;

static void lsm6dxxWriteRegister(const  busDevice_t *dev, lsm6dxxRegister_e registerID, uint8_t value, unsigned delayMs)
{
    busWrite(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void lsm6dxxWriteRegisterBits(const  busDevice_t *dev, lsm6dxxRegister_e registerID, lsm6dxxConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busRead(dev, registerID, &newValue)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        lsm6dxxWriteRegister(dev, registerID, newValue, delayMs);
    }
}

static uint8_t getLsmDlpfBandwidth(gyroDev_t *gyro)
{
    switch(gyro->lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return LSM6DXX_VAL_CTRL6_C_FTYPE_201HZ;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return LSM6DXX_VAL_CTRL6_C_FTYPE_300HZ;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return LSM6DXX_VAL_CTRL6_C_FTYPE_603HZ;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return LSM6DXX_VAL_CTRL6_C_FTYPE_603HZ;
    }
    return 0;
}

static void lsm6dsv16xConfig(gyroDev_t *gyro)
{
    busDevice_t *dev = gyro->busDev;
    const gyroFilterAndRateConfig_t *config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    // Software reset
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL3_C, BIT(0), 100);

    // Enable Block Data Update (BDU) and auto-increment address
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL3_C, 
                         BIT(6) |  // BDU - Block Data Update
                         BIT(2), 1); // IF_INC - Auto-increment address

    // 配置HAODR模式 - 使用正常ODR编码 (HAODR_SEL = 00)
    // 这允许我们使用标准的ODR值如1920Hz
    // 寄存器0x62是HAODR_CFG寄存器
    lsm6dxxWriteRegister(dev, LSM6DSV16X_REG_HAODR_CFG, LSM6DSV16X_VAL_HAODR_CFG_NORMAL, 1);

    // 配置加速度计: 1920Hz ODR, ±16G量程, 高性能模式
    // CTRL1_XL[7:4] = OP_MODE_XL (0=HIGH_PERF), CTRL1_XL[3:0] = ODR_XL (0x0A=1920Hz)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL1_XL, 
                         (0x00 << 4) |  // OP_MODE_XL = 0 (HIGH_PERF)
                         LSM6DSV16X_VAL_CTRL1_XL_ODR1920, 1); // ODR_XL = 0x0A (1920Hz)

    // 配置陀螺仪: 1920Hz ODR, ±2000dps量程, 高性能模式
    // CTRL2_G[7:4] = OP_MODE_G (0=HIGH_PERF), CTRL2_G[3:0] = ODR_G (0x0A=1920Hz)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL2_G, 
                         (0x00 << 4) |  // OP_MODE_G = 0 (HIGH_PERF)
                         LSM6DSV16X_VAL_CTRL2_G_ODR1920, 1); // ODR_G = 0x0A (1920Hz)

    // 设置加速度计量程为±16g (CTRL8_XL)
    // CTRL8_XL[1:0] = FS_XL (0x03 = ±16g)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL8_XL, 
                         LSM6DSV16X_VAL_CTRL8_FS_XL_16G, 1);  // FS_XL = 11 (±16g)

    // 设置陀螺仪量程为±2000dps (CTRL6_C)
    // CTRL6_C[3:0] = FS_G (0x04 = ±2000dps)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL6_C, 
                         LSM6DSV16X_VAL_CTRL6_FS_G_2000DPS, 1);  // FS_G = 0100 (±2000dps)

    // 启用陀螺仪LPF1滤波器 (CTRL7_G)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL7_G, 
                         BIT(0), 1);  // LPF1_G_EN

    // 配置中断: 数据就绪脉冲模式 (CTRL4_C)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL4_C, 
                         BIT(1), 1);  // DRDY_PULSED

    // 启用INT1引脚的陀螺仪数据就绪中断 (INT1_CTRL)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_INT1_CTRL, 
                         BIT(1), 1);  // INT1_DRDY_G

    // 禁用I2C/I3C接口，仅使用SPI (IF_CFG寄存器0x03)
    // LSM6DSV16X使用IF_CFG寄存器而不是CTRL9_XL
    lsm6dxxWriteRegister(dev, LSM6DSV16X_REG_IF_CFG, 
                         LSM6DSV16X_VAL_IF_CFG_I2C_I3C_DISABLE, 1);  // I2C_I3C_DISABLE

    // 验证配置 - 读回关键寄存器进行验证
    uint8_t who_am_i, ctrl1_xl, ctrl2_g, ctrl6_c, ctrl8_xl, haodr_cfg, if_cfg;
    busRead(dev, LSM6DXX_REG_WHO_AM_I, &who_am_i);
    busRead(dev, LSM6DXX_REG_CTRL1_XL, &ctrl1_xl);
    busRead(dev, LSM6DXX_REG_CTRL2_G, &ctrl2_g);
    busRead(dev, LSM6DXX_REG_CTRL6_C, &ctrl6_c);
    busRead(dev, LSM6DXX_REG_CTRL8_XL, &ctrl8_xl);
    busRead(dev, LSM6DSV16X_REG_HAODR_CFG, &haodr_cfg);
    busRead(dev, LSM6DSV16X_REG_IF_CFG, &if_cfg);

    // 使用INAV的调试系统输出配置验证信息
    // 这些值可以通过CLI的debug_mode = ACC查看
    // 从DEBUG[3]开始显示，避免与其他数据冲突
    DEBUG_SET(DEBUG_ACC, 3, who_am_i);      // WHO_AM_I (应该是0x70)
    DEBUG_SET(DEBUG_ACC, 4, ctrl1_xl);     // 加速度计配置 (应该是0x0A)
    DEBUG_SET(DEBUG_ACC, 5, ctrl2_g);      // 陀螺仪配置 (应该是0x0A)
    DEBUG_SET(DEBUG_ACC, 6, (ctrl6_c << 8) | ctrl8_xl); // 量程配置
    DEBUG_SET(DEBUG_ACC, 7, (haodr_cfg << 8) | if_cfg); // HAODR和接口配置

    busSetSpeed(dev, BUS_SPEED_FAST);
}

static void lsm6dxxConfig(gyroDev_t *gyro)
{ 
    // Dispatch to chip-specific configuration based on detected chip ID
    if (lsm6dID == LSM6DSV16X_CHIP_ID || lsm6dID == LSM6DSV32X_CHIP_ID || lsm6dID == LSM6DSK320X_CHIP_ID) {
        lsm6dsv16xConfig(gyro);
        return;
    }

    // LSM6DSO/LSM6DSL configuration
    busDevice_t * dev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);
    // Reset the device (wait 100ms before continuing config)
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL3_C, LSM6DXX_MASK_CTRL3_C_RESET, BIT(0), 100);

    // Configure data ready pulsed mode
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_COUNTER_BDR1, LSM6DXX_MASK_COUNTER_BDR1, LSM6DXX_VAL_COUNTER_BDR1_DDRY_PM, 0);
 
    // Configure interrupt pin 1 for gyro data ready only
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_INT1_CTRL, LSM6DXX_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_INT2_CTRL, LSM6DXX_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF2 output (default with ODR/4 cutoff)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL1_XL, (LSM6DXX_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DXX_VAL_CTRL1_XL_16G << 2) | (LSM6DXX_VAL_CTRL1_XL_LPF2 << 1), 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL2_G, (LSM6DXX_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DXX_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL3_C, LSM6DXX_MASK_CTRL3_C, (LSM6DXX_VAL_CTRL3_C_H_LACTIVE | LSM6DXX_VAL_CTRL3_C_PP_OD | LSM6DXX_VAL_CTRL3_C_SIM | LSM6DXX_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // enable accelerometer high performane mode; enable gyro LPF1
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL4_C, LSM6DXX_MASK_CTRL4_C, (LSM6DXX_VAL_CTRL4_C_DRDY_MASK | LSM6DXX_VAL_CTRL4_C_LPF1_SEL_G), 1);

 

    // Configure control register 6
    // disable I2C interface; set gyro LPF1 cutoff according to gyro_hardware_lpf setting
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL6_C, (lsm6dID == LSM6DSO_CHIP_ID? LSM6DXX_MASK_CTRL6_C:LSM6DSL_MASK_CTRL6_C), getLsmDlpfBandwidth(gyro), 1);

    // Configure control register 7
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL7_G, LSM6DXX_MASK_CTRL7_G, LSM6DXX_VAL_CTRL7_G_HPM_G_16, 1);

    // Configure control register 9
    // disable I3C interface
    if(lsm6dID == LSM6DSO_CHIP_ID)
    {
        lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL9_XL, LSM6DXX_MASK_CTRL9_XL, LSM6DXX_VAL_CTRL9_XL_I3C_DISABLE, 1);
    }
    else
    {
        lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL9_XL, LSM6DSV16X_MASK_CTRL9_XL, LSM6DSV16X_VAL_CTRL9_I2C_I3C_DISABLE, 1);
    }

    busSetSpeed(dev, BUS_SPEED_FAST);
}



static bool lsm6dxxDetect(busDevice_t * dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;
    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);
    do {
        delay(150);

        busRead(dev, LSM6DXX_REG_WHO_AM_I, &tmp);

        switch (tmp) {
            case LSM6DSO_CHIP_ID:
            case LSM6DSL_CHIP_ID: 
            case LSM6DSV16X_CHIP_ID:
            case LSM6DSV32X_CHIP_ID:
            case LSM6DSK320X_CHIP_ID:
                 lsm6dID = tmp;
                // Compatible chip detected
                return true;
            default:
                // Retry detection
                break;
        }
    } while (attemptsRemaining--);

    return false;
}

static void lsm6dxxSpiGyroInit(gyroDev_t *gyro)
{   
    lsm6dxxConfig(gyro);
}

static void lsm6dxxSpiAccInit(accDev_t *acc)
{
    // For LSM6DSV16X/LSM6DSV32X: ±16G sensor scale
    // ST官方转换因子: 0.488 mg/LSB for ±16g
    // INAV acc_1G格式: 1G = 1000mg, 所以 acc_1G = 1000/0.488 ≈ 2049
    if (lsm6dID == LSM6DSV16X_CHIP_ID || lsm6dID == LSM6DSV32X_CHIP_ID || lsm6dID == LSM6DSK320X_CHIP_ID) {
        acc->acc_1G = 2049;   // LSM6DSV16X/LSM6DSV32X: 0.488 mg/LSB for ±16g
    } else {
        acc->acc_1G = 2048;   // LSM6DSO/DSL: 传统值
    }
}

static bool lsm6dxxAccRead(accDev_t *acc)
{
    uint8_t data[6];
    const bool ack = busReadBuf(acc->busDev, LSM6DXX_REG_OUTX_L_A, data, 6);
    if (!ack) {
        return false;
    }
    
    // 加速度计不进行轴向交换
    acc->ADCRaw[X] = (float) int16_val_little_endian(data, 0);
    acc->ADCRaw[Y] = (float) int16_val_little_endian(data, 1);
    acc->ADCRaw[Z] = (float) int16_val_little_endian(data, 2);
    
    return true; 
}

static bool lsm6dxxGyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];
    const bool ack = busReadBuf(gyro->busDev, LSM6DXX_REG_OUTX_L_G, data, 6);
    if (!ack) {
        return false;
    }
    
    // 陀螺仪不需要轴向交换
    gyro->gyroADCRaw[X] = (float) int16_val_little_endian(data, 0);
    gyro->gyroADCRaw[Y] = (float) int16_val_little_endian(data, 1);
    gyro->gyroADCRaw[Z] = (float) int16_val_little_endian(data, 2);
    return true;
}

// Init Gyro first,then Acc
bool lsm6dGyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_SPI, DEVHW_LSM6D, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!lsm6dxxDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    lsm6DContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0xD6;

    gyro->initFn = lsm6dxxSpiGyroInit;
    gyro->readFn = lsm6dxxGyroRead;
    gyro->intStatusFn = gyroCheckDataReady;
    
    gyro->scale = 1.0f / 14.286f; // 2000 dps: 70 mdps/LSB (from datasheet)
    
    gyro->gyroAlign = gyro->busDev->param;  // 使用 target.h 中定义的对齐设置
    return true;

}
bool lsm6dAccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_SPI, DEVHW_LSM6D, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    lsm6DContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0xD6) {
        return false;
    }
    acc->initFn = lsm6dxxSpiAccInit;
    acc->readFn = lsm6dxxAccRead;
    acc->accAlign = acc->busDev->param;

    return true;
}



#endif
