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

    // 复位设备（等待100ms后继续配置）
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL3_C, LSM6DXX_MASK_CTRL3_C_RESET, BIT(0), 100);

    // 配置数据就绪脉冲模式
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_COUNTER_BDR1, LSM6DXX_MASK_COUNTER_BDR1, LSM6DXX_VAL_COUNTER_BDR1_DDRY_PM, 0);

    // 配置中断引脚1为陀螺仪数据就绪
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_INT1_CTRL, LSM6DXX_VAL_INT1_CTRL, 1);

    // 禁用中断引脚2
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_INT2_CTRL, LSM6DXX_VAL_INT2_CTRL, 1);

    // 配置加速度计 - LSM6DSV16X 特定
    // LSM6DSV16X CTRL1_XL 寄存器布局:
    //   bit[7:4] = ODR_XL[3:0] (输出数据率)
    //   bit[3:2] = FS[1:0]_XL  (量程: 00=2g, 01=4g, 10=8g, 11=16g)
    //   bit[1]   = LPF2_XL_EN  (LPF2使能)
    //   bit[0]   = 保留
    // 960Hz ODR (0x09), 16G scale (FS=11), LPF2 enabled
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL1_XL, 
                         (LSM6DSV16X_VAL_CTRL1_XL_ODR960 << 4) |  // ODR 在 bit[7:4]
                         (LSM6DSV16X_VAL_CTRL1_XL_16G << 2) |     // FS_XL 在 bit[3:2]
                         (LSM6DXX_VAL_CTRL1_XL_LPF2 << 1), 1);    // LPF2_XL_EN 在 bit[1]

    // 配置陀螺仪 - LSM6DSV16X 特定
    // LSM6DSV16X CTRL2_G 寄存器布局:
    //   bit[7:4] = ODR_G[3:0]  (输出数据率)
    //   bit[3:1] = FS[2:0]_G   (量程: 000=250dps, 001=500dps, 010=1000dps, 011=2000dps, 100=4000dps)
    //   bit[0]   = 保留
    // 注意: LSM6DSV16X 的 FS_G 是 3 位，位于 bit[3:1]，需要左移 1 位！
    // 7680Hz ODR (0x0B), 2000dps scale (FS=011)
    lsm6dxxWriteRegister(dev, LSM6DXX_REG_CTRL2_G, 
                         (LSM6DSV16X_VAL_CTRL2_G_ODR7680 << 4) |  // ODR 在 bit[7:4]
                         (LSM6DSV16X_VAL_CTRL2_G_2000DPS << 1), 1); // FS_G 在 bit[3:1]，左移1位！

    // 配置控制寄存器3
    // 读取时锁存LSB/MSB；中断引脚高电平有效；中断引脚推挽输出；4线SPI模式；使能自动地址递增
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL3_C, LSM6DXX_MASK_CTRL3_C, 
                             (LSM6DXX_VAL_CTRL3_C_H_LACTIVE | 
                              LSM6DXX_VAL_CTRL3_C_PP_OD | 
                              LSM6DXX_VAL_CTRL3_C_SIM | 
                              LSM6DXX_VAL_CTRL3_C_IF_INC), 1);

    // 配置控制寄存器4
    // 使能加速度计高性能模式；使能陀螺仪LPF1
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL4_C, LSM6DXX_MASK_CTRL4_C, 
                             (LSM6DXX_VAL_CTRL4_C_DRDY_MASK | 
                              LSM6DXX_VAL_CTRL4_C_I2C_DISABLE | 
                              LSM6DXX_VAL_CTRL4_C_LPF1_SEL_G), 1);

    // 配置控制寄存器6 - LSM6DSV16X 没有 XL_HM_MODE 位
    // 使用 LSM6DSL 掩码（无 XL_HM_MODE）；根据 gyro_hardware_lpf 设置陀螺仪 LPF1 截止频率
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL6_C, LSM6DSL_MASK_CTRL6_C, getLsmDlpfBandwidth(gyro), 1);

    // 配置控制寄存器7 - LSM6DSV16X
    // 注意: LSM6DSV16X 的 CTRL7_G 位布局可能与 LSM6DSOX 不同
    // 暂时禁用陀螺仪高通滤波器，避免可能的漂移问题
    // bit 7: G_HM_MODE = 0 (启用高性能模式)
    // bit 6: HP_EN_G = 0 (禁用高通滤波器)
    // bit 5:4: HPM[1:0]_G = 00 (HPF截止频率16mHz，但HPF已禁用)
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL7_G, LSM6DXX_MASK_CTRL7_G, 
                             LSM6DXX_VAL_CTRL7_G_HPM_G_16, 1);  // 只设置HPM，不启用HP_EN_G

    // 配置控制寄存器9 - 禁用 I2C 和 I3C 接口
    // LSM6DSV16X 使用 bit 0 而不是 bit 1 来禁用 I2C/I3C
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL9_XL, LSM6DSV16X_MASK_CTRL9_XL, 
                             LSM6DSV16X_VAL_CTRL9_I2C_I3C_DISABLE, 1);

    busSetSpeed(dev, BUS_SPEED_FAST);
}

static void lsm6dxxConfig(gyroDev_t *gyro)
{ 
    // Dispatch to chip-specific configuration based on detected chip ID
    if (lsm6dID == LSM6DSV16X_CHIP_ID) {
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
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL4_C, LSM6DXX_MASK_CTRL4_C, (LSM6DXX_VAL_CTRL4_C_DRDY_MASK | LSM6DXX_VAL_CTRL4_C_I2C_DISABLE | LSM6DXX_VAL_CTRL4_C_LPF1_SEL_G), 1);

 

    // Configure control register 6
    // disable I2C interface; set gyro LPF1 cutoff according to gyro_hardware_lpf setting
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL6_C, (lsm6dID == LSM6DSO_CHIP_ID? LSM6DXX_MASK_CTRL6_C:LSM6DSL_MASK_CTRL6_C), (LSM6DXX_VAL_CTRL6_C_XL_HM_MODE | getLsmDlpfBandwidth(gyro)), 1);

    // Configure control register 7
    lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL7_G, LSM6DXX_MASK_CTRL7_G, (LSM6DXX_VAL_CTRL7_G_HP_EN_G | LSM6DXX_VAL_CTRL7_G_HPM_G_16), 1);

    // Configure control register 9
    // disable I3C interface
    if(lsm6dID == LSM6DSO_CHIP_ID)
    {
        lsm6dxxWriteRegisterBits(dev, LSM6DXX_REG_CTRL9_XL, LSM6DXX_MASK_CTRL9_XL, LSM6DXX_VAL_CTRL9_XL_I3C_DISABLE, 1);
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
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
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
    gyro->scale = 1.0f / 16.4f; // 2000 dps
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