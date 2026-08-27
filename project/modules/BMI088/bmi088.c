#include "bmi088.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*---------- SPI 读写 (BMI088 Accel 需要 dummy byte) ----------*/

static void BMI088_Acc_ReadReg(BMI088_Instance *bmi088, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};
    tx[0] = 0x80 | reg;
    SPI_Transfer(bmi088->spi_acc, tx, rx, len + 2);
    memcpy(buf, rx + 2, len);
}

static void BMI088_Acc_WriteReg(BMI088_Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    uint8_t rx[2] = {0};
    SPI_Transfer(bmi088->spi_acc, tx, rx, 2);
}

static void BMI088_Gyro_ReadReg(BMI088_Instance *bmi088, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};
    tx[0] = 0x80 | reg;
    SPI_Transfer(bmi088->spi_gyro, tx, rx, len + 1);
    memcpy(buf, rx + 1, len);
}

static void BMI088_Gyro_WriteReg(BMI088_Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    uint8_t rx[2] = {0};
    SPI_Transfer(bmi088->spi_gyro, tx, rx, 2);
}

/*---------- 量程设置 ----------*/

void BMI088_Set_Accel_Range(BMI088_Instance *bmi088, BMI088_AccRange_e range)
{
    if (!bmi088 || range > BMI088_ACC_RANGE_24G_E) return;
    bmi088->accel_range = range;
    BMI088_Acc_WriteReg(bmi088, BMI088_ACC_RANGE, range);
    DWT_Delay_ms(1);
}

void BMI088_Set_Gyro_Range(BMI088_Instance *bmi088, BMI088_GyroRange_e range)
{
    if (!bmi088 || range > BMI088_GYRO_RANGE_125_E) return;
    bmi088->gyro_range = range;
    BMI088_Gyro_WriteReg(bmi088, BMI088_GYRO_RANGE, 4 - range);
    DWT_Delay_ms(1);
}

/*---------- 初始化 ----------*/

static uint8_t BMI088_Acc_Init(BMI088_Instance *bmi088)
{
    uint8_t id = 0;

    // BMI088 Accel 上电后默认 I2C 模式，需要 CSB1 上升沿切换到 SPI 模式。
    // 执行一次 SPI 读操作（拉低 CS 再拉高），产生上升沿触发模式切换。
    // 返回值无效，仅用于触发切换。
    BMI088_Acc_ReadReg(bmi088, BMI088_ACC_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);

    // 软复位，将 Accel 所有寄存器恢复默认值
    BMI088_Acc_WriteReg(bmi088, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    DWT_Delay_ms(80);

    // 软复位后读取 CHIP_ID 验证 SPI 通信正常
    BMI088_Acc_ReadReg(bmi088, BMI088_ACC_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);
    BMI088_Acc_ReadReg(bmi088, BMI088_ACC_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);
    if (id != BMI088_ACC_CHIP_ID_VALUE) return 1;

    // 使能 Accel 测量
    BMI088_Acc_WriteReg(bmi088, BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE);
    DWT_Delay_ms(1);
    BMI088_Acc_WriteReg(bmi088, BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE);
    DWT_Delay_ms(1);

    // 配置 ODR 800Hz，正常带宽
    // ACC_CONF bits[7:4]=acc_bwp=0x0A(Normal), bits[3:0]=acc_odr=0x0B(800Hz) → 0xAB
    BMI088_Acc_WriteReg(bmi088, BMI088_ACC_CONF, BMI088_ACC_BWP_NORMAL | BMI088_ACC_ODR_800_HZ);
    DWT_Delay_ms(1);

    return 0;
}

static uint8_t BMI088_Gyro_Init(BMI088_Instance *bmi088)
{
    uint8_t id = 0;

    BMI088_Gyro_ReadReg(bmi088, BMI088_GYRO_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);

    // 软复位，将 Gyro 所有寄存器恢复默认值
    BMI088_Gyro_WriteReg(bmi088, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    DWT_Delay_ms(80);

    // 读取 CHIP_ID 验证 SPI 通信正常（Gyro 由 PS 引脚直接选择 SPI 模式，无需切换）
    BMI088_Gyro_ReadReg(bmi088, BMI088_GYRO_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);
    if (id != BMI088_GYRO_CHIP_ID_VALUE) return 1;

    // 配置量程 (枚举值与寄存器编码反向: 0x00=2000, 0x04=125)
    BMI088_Gyro_WriteReg(bmi088, BMI088_GYRO_RANGE, 4 - bmi088->gyro_range);
    DWT_Delay_ms(1);

    // 配置带宽 230Hz，加 ODR 使能位（0x80）
    BMI088_Gyro_WriteReg(bmi088, BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BW_230_HZ | 0x80);
    DWT_Delay_ms(1);

    // 设置正常模式，使能数据就绪中断
    BMI088_Gyro_WriteReg(bmi088, BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
    DWT_Delay_ms(1);
    BMI088_Gyro_WriteReg(bmi088, BMI088_GYRO_CTRL, BMI088_GYRO_DRDY_ON);
    DWT_Delay_ms(1);

    return 0;
}

/*---------- 公有 API ----------*/

BMI088_Instance *BMI088_Register(BMI088_Init_Config_s *config)
{
    BMI088_Instance *bmi088 = (BMI088_Instance *)malloc(sizeof(BMI088_Instance));
    if (!bmi088) return NULL;
    memset(bmi088, 0, sizeof(BMI088_Instance));

    bmi088->spi_acc = SPI_Register(&config->spi_acc_config);
    bmi088->spi_gyro = SPI_Register(&config->spi_gyro_config);
    if (!bmi088->spi_acc || !bmi088->spi_gyro) {
        free(bmi088);
        return NULL;
    }

    bmi088->accel_range = config->accel_range;
    bmi088->gyro_range = config->gyro_range;

    // 椭球拟合校准参数
    bmi088->use_ellipsoid_cal = 0;
    memset(bmi088->accel_offset, 0, sizeof(bmi088->accel_offset));
    memset(bmi088->accel_M, 0, sizeof(bmi088->accel_M));
    // 检查是否提供了校准参数 (M[0] != 0 表示有有效校准)
    if (config->accel_M[0] != 0.0f) {
        bmi088->use_ellipsoid_cal = 1;
        memcpy(bmi088->accel_offset, config->accel_offset, sizeof(bmi088->accel_offset));
        memcpy(bmi088->accel_M, config->accel_M, sizeof(bmi088->accel_M));
    }

    if (BMI088_Acc_Init(bmi088) != 0) {
        free(bmi088);
        return NULL;
    }
    if (BMI088_Gyro_Init(bmi088) != 0) {
        free(bmi088);
        return NULL;
    }

    BMI088_Set_Accel_Range(bmi088, config->accel_range);
    BMI088_Set_Gyro_Range(bmi088, config->gyro_range);

    return bmi088;
}

void BMI088_Read_Accel(BMI088_Instance *bmi088)
{
    if (!bmi088) return;
    uint8_t buf[6] = {0};
    BMI088_Acc_ReadReg(bmi088, BMI088_ACCEL_XOUT_L, buf, 6);

    float bmi088_accel_scale = (1 << bmi088->accel_range) * BMI088_ACCEL_3G_SEN;
    float raw_x = (int16_t)(buf[1] << 8 | buf[0]) * bmi088_accel_scale;
    float raw_y = (int16_t)(buf[3] << 8 | buf[2]) * bmi088_accel_scale;
    float raw_z = (int16_t)(buf[5] << 8 | buf[4]) * bmi088_accel_scale;

    if (bmi088->use_ellipsoid_cal) {
        // 椭球拟合校准: corrected = M × (raw - offset)
        float dx = raw_x - bmi088->accel_offset[0];
        float dy = raw_y - bmi088->accel_offset[1];
        float dz = raw_z - bmi088->accel_offset[2];
        float *M = bmi088->accel_M;
        bmi088->accel.x = M[0]*dx + M[1]*dy + M[2]*dz;
        bmi088->accel.y = M[3]*dx + M[4]*dy + M[5]*dz;
        bmi088->accel.z = M[6]*dx + M[7]*dy + M[8]*dz;
    } else {
        // 不校准，直接使用原始值
        bmi088->accel.x = raw_x;
        bmi088->accel.y = raw_y;
        bmi088->accel.z = raw_z;
    }
}

void BMI088_Read_Gyro(BMI088_Instance *bmi088)
{
    if (!bmi088) return;
    uint8_t buf[6] = {0};
    BMI088_Gyro_ReadReg(bmi088, BMI088_GYRO_X_L, buf, 6);

    float bmi088_gyro_scale = (1 << bmi088->gyro_range) * BMI088_GYRO_125_SEN;
    bmi088->gyro.x = (int16_t)(buf[1] << 8 | buf[0]) * bmi088_gyro_scale - bmi088->gyro_offset[0];
    bmi088->gyro.y = (int16_t)(buf[3] << 8 | buf[2]) * bmi088_gyro_scale - bmi088->gyro_offset[1];
    bmi088->gyro.z = (int16_t)(buf[5] << 8 | buf[4]) * bmi088_gyro_scale - bmi088->gyro_offset[2];
}

void BMI088_Read_Temp(BMI088_Instance *bmi088)
{
    if (!bmi088) return;
    uint8_t buf[2] = {0};
    BMI088_Acc_ReadReg(bmi088, BMI088_TEMP_M, buf, 2);
    int16_t raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023) raw -= 2048;
    bmi088->temperature = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088_Read_All(BMI088_Instance *bmi088)
{
    BMI088_Read_Accel(bmi088);
    BMI088_Read_Gyro(bmi088);
    BMI088_Read_Temp(bmi088);
}

void BMI088_Calibrate(BMI088_Instance *bmi088)
{
    if (!bmi088) return;

    const uint16_t cali_times = 6000;
    float gyro_sum[3] = {0};
    float gyroMax[3], gyroMin[3];
    float gyroDiff[3];
    uint32_t start_tick = DWT_GetTimeline_us();
    uint8_t pass = 0;

    do {
        gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0;
        gyroMax[0] = gyroMax[1] = gyroMax[2] = 0;
        gyroMin[0] = gyroMin[1] = gyroMin[2] = 0;

        for (uint16_t i = 0; i < cali_times; i++) {
            BMI088_Read_Gyro(bmi088);

            if (i == 0) {
                gyroMax[0] = gyroMin[0] = bmi088->gyro.x;
                gyroMax[1] = gyroMin[1] = bmi088->gyro.y;
                gyroMax[2] = gyroMin[2] = bmi088->gyro.z;
            } else {
                if (bmi088->gyro.x > gyroMax[0]) gyroMax[0] = bmi088->gyro.x;
                if (bmi088->gyro.x < gyroMin[0]) gyroMin[0] = bmi088->gyro.x;
                if (bmi088->gyro.y > gyroMax[1]) gyroMax[1] = bmi088->gyro.y;
                if (bmi088->gyro.y < gyroMin[1]) gyroMin[1] = bmi088->gyro.y;
                if (bmi088->gyro.z > gyroMax[2]) gyroMax[2] = bmi088->gyro.z;
                if (bmi088->gyro.z < gyroMin[2]) gyroMin[2] = bmi088->gyro.z;
            }

            gyro_sum[0] += bmi088->gyro.x;
            gyro_sum[1] += bmi088->gyro.y;
            gyro_sum[2] += bmi088->gyro.z;

            DWT_Delay_ms(1);
        }

        // stability checks (gyro should be stable when stationary)
        gyroDiff[0] = gyroMax[0] - gyroMin[0];
        gyroDiff[1] = gyroMax[1] - gyroMin[1];
        gyroDiff[2] = gyroMax[2] - gyroMin[2];

        pass = (gyroDiff[0] < 0.15f) &&
               (gyroDiff[1] < 0.15f) &&
               (gyroDiff[2] < 0.15f);

    } while (!pass && (DWT_GetTimeline_us() - start_tick) < 12000000);  // 12s timeout

    // set gyro calibration results
    for (uint8_t i = 0; i < 3; i++)
        bmi088->gyro_offset[i] = gyro_sum[i] / cali_times;
}
