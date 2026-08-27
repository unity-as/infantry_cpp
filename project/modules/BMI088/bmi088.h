#ifndef BMI088_H
#define BMI088_H

#include "bsp_spi.h"
#include "bsp_dwt.h"
#include <stdint.h>

/*---------- 加速度计寄存器 ----------*/

#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_CHIP_ID_VALUE    0x1E

#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03

#define BMI088_ACCEL_XOUT_L         0x12
#define BMI088_ACCEL_XOUT_M         0x13
#define BMI088_ACCEL_YOUT_L         0x14
#define BMI088_ACCEL_YOUT_M         0x15
#define BMI088_ACCEL_ZOUT_L         0x16
#define BMI088_ACCEL_ZOUT_M         0x17

#define BMI088_TEMP_M               0x22
#define BMI088_TEMP_L               0x23

#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41

#define BMI088_INT1_IO_CTRL         0x53
#define BMI088_INT_MAP_DATA         0x58

#define BMI088_ACC_SELF_TEST        0x6D

#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E
#define BMI088_ACC_SOFTRESET_VALUE  0xB6

/*---------- 陀螺仪寄存器 ----------*/

#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F

#define BMI088_GYRO_X_L             0x02
#define BMI088_GYRO_X_H             0x03
#define BMI088_GYRO_Y_L             0x04
#define BMI088_GYRO_Y_H             0x05
#define BMI088_GYRO_Z_L             0x06
#define BMI088_GYRO_Z_H             0x07

#define BMI088_GYRO_INT_STAT_1      0x0A
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_SOFTRESET       0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6
#define BMI088_GYRO_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP  0x18
#define BMI088_GYRO_SELF_TEST       0x3C

/*---------- 配置值 ----------*/

// 加速度计量程
#define BMI088_ACC_RANGE_3G         0x00
#define BMI088_ACC_RANGE_6G         0x01
#define BMI088_ACC_RANGE_12G        0x02
#define BMI088_ACC_RANGE_24G        0x03

// 加速度计 ODR
#define BMI088_ACC_ODR_12_5_HZ      0x05
#define BMI088_ACC_ODR_25_HZ        0x06
#define BMI088_ACC_ODR_50_HZ        0x07
#define BMI088_ACC_ODR_100_HZ       0x08
#define BMI088_ACC_ODR_200_HZ       0x09
#define BMI088_ACC_ODR_400_HZ       0x0A
#define BMI088_ACC_ODR_800_HZ       0x0B
#define BMI088_ACC_ODR_1600_HZ      0x0C

// 加速度计带宽 (ACC_CONF bits[7:4], 合法值必须左移4位)
#define BMI088_ACC_BWP_OSR4         (0x08 << 4) // 0x80
#define BMI088_ACC_BWP_OSR2         (0x09 << 4) // 0x90
#define BMI088_ACC_BWP_NORMAL       (0x0A << 4) // 0xA0

// 陀螺仪量程
#define BMI088_GYRO_RANGE_2000      0x00
#define BMI088_GYRO_RANGE_1000      0x01
#define BMI088_GYRO_RANGE_500       0x02
#define BMI088_GYRO_RANGE_250       0x03
#define BMI088_GYRO_RANGE_125       0x04

// 陀螺仪带宽
#define BMI088_GYRO_BW_532_HZ       0x00
#define BMI088_GYRO_BW_230_HZ       0x01
#define BMI088_GYRO_BW_116_HZ       0x02
#define BMI088_GYRO_BW_47_HZ        0x03
#define BMI088_GYRO_BW_23_HZ        0x04
#define BMI088_GYRO_BW_12_HZ        0x05
#define BMI088_GYRO_BW_64_HZ        0x06
#define BMI088_GYRO_BW_32_HZ        0x07

// 电源模式
#define BMI088_ACC_PWR_ACTIVE       0x00
#define BMI088_ACC_PWR_SUSPEND      0x03
#define BMI088_ACC_ENABLE           0x04
#define BMI088_ACC_DISABLE          0x00

#define BMI088_GYRO_NORMAL_MODE     0x00
#define BMI088_GYRO_SUSPEND_MODE    0x80
#define BMI088_GYRO_DRDY_ON        0x80

/*---------- 灵敏度系数 ----------*/

#define BMI088_ACCEL_3G_SEN         0.0008974358974f
#define BMI088_ACCEL_6G_SEN         0.00179443359375f
#define BMI088_ACCEL_12G_SEN        0.0035888671875f
#define BMI088_ACCEL_24G_SEN        0.007177734375f

#define BMI088_GYRO_2000_SEN        0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN        0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN         0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN         0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN         0.000066579027251980956150958662738366f

#define BMI088_TEMP_FACTOR          0.125f
#define BMI088_TEMP_OFFSET          23.0f

/*---------- 配置枚举 ----------*/

typedef enum {
    BMI088_ACC_RANGE_3G_E = 0,
    BMI088_ACC_RANGE_6G_E,
    BMI088_ACC_RANGE_12G_E,
    BMI088_ACC_RANGE_24G_E,
} BMI088_AccRange_e;

typedef enum {
    BMI088_GYRO_RANGE_125_E = 0,
    BMI088_GYRO_RANGE_250_E,
    BMI088_GYRO_RANGE_500_E,
    BMI088_GYRO_RANGE_1000_E,
    BMI088_GYRO_RANGE_2000_E,
} BMI088_GyroRange_e;

/*---------- 实例结构体 ----------*/

typedef struct {
    // SPI 接口
    SPI_Instance *spi_acc;
    SPI_Instance *spi_gyro;

    // 量程配置
    BMI088_AccRange_e accel_range;
    BMI088_GyroRange_e gyro_range;

    // 陀螺仪校准参数 (运行时校准)
    float gyro_offset[3];

    // 加速度计椭球拟合校准 (离线生成)
    uint8_t use_ellipsoid_cal;   // 1=使用椭球校准, 0=不校准
    float accel_offset[3];       // 加速度偏移
    float accel_M[9];            // 3×3 修正矩阵 (含 scale + 非正交性)

    // 输出数据 (单位: m/s², rad/s, °C)
    struct {
        float x, y, z;
    } accel;
    struct {
        float x, y, z;
    } gyro;
    float temperature;
} BMI088_Instance;

/*---------- 初始化配置 ----------*/

typedef struct {
    SPI_Init_Config_s spi_acc_config;
    SPI_Init_Config_s spi_gyro_config;
    BMI088_AccRange_e accel_range;
    BMI088_GyroRange_e gyro_range;

    // 椭球拟合校准参数 (离线生成，不填则不校准)
    float accel_offset[3];       // 加速度偏移
    float accel_M[9];            // 3×3 修正矩阵
} BMI088_Init_Config_s;

/*---------- API ----------*/

BMI088_Instance *BMI088_Register(BMI088_Init_Config_s *config);
void BMI088_Set_Accel_Range(BMI088_Instance *bmi088, BMI088_AccRange_e range);
void BMI088_Set_Gyro_Range(BMI088_Instance *bmi088, BMI088_GyroRange_e range);
void BMI088_Read_Accel(BMI088_Instance *bmi088);
void BMI088_Read_Gyro(BMI088_Instance *bmi088);
void BMI088_Read_Temp(BMI088_Instance *bmi088);
void BMI088_Read_All(BMI088_Instance *bmi088);
void BMI088_Calibrate(BMI088_Instance *bmi088);

#endif
