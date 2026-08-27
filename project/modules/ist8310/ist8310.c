#include "ist8310.h"
#include <stdlib.h>
#include <string.h>

/*---------- 内部辅助函数 ----------*/

static void IST8310_Reset(IST8310_Instance *ist8310)
{
    // 拉低复位引脚 50ms, 再拉高 50ms
    GPIO_WritePin(ist8310->rst, GPIO_PIN_RESET);
    HAL_Delay(50);
    GPIO_WritePin(ist8310->rst, GPIO_PIN_SET);
    HAL_Delay(50);
}

static uint8_t IST8310_ReadReg(IST8310_Instance *ist8310, uint8_t reg)
{
    uint8_t data = 0;
    HARD_I2C_Mem_Read(ist8310->i2c, reg, &data, 1);
    return data;
}

static void IST8310_WriteReg(IST8310_Instance *ist8310, uint8_t reg, uint8_t data)
{
    HARD_I2C_Mem_Write(ist8310->i2c, reg, &data, 1);
}

/*---------- 公有 API ----------*/

IST8310_Instance *IST8310_Register(IST8310_Init_Config_s *config)
{
    IST8310_Instance *ist8310 = (IST8310_Instance *)malloc(sizeof(IST8310_Instance));
    if (!ist8310) return NULL;
    memset(ist8310, 0, sizeof(IST8310_Instance));

    // 注册 I2C 和 GPIO 实例
    ist8310->i2c = HARD_I2C_Init(&config->i2c_config);
    ist8310->rst = GPIO_Register(&config->rst_config);
    if (!ist8310->i2c || !ist8310->rst) {
        free(ist8310);
        return NULL;
    }

    // 硬件复位
    IST8310_Reset(ist8310);

    // WHO_AM_I 校验
    uint8_t id = IST8310_ReadReg(ist8310, IST8310_REG_WHO_AM_I);
    if (id != IST8310_WHO_AM_I_VALUE) {
        free(ist8310);
        return NULL;
    }

    // 配置: 200Hz ODR, 使能中断
    IST8310_WriteReg(ist8310, IST8310_REG_CNTL2, IST8310_INT_ENABLE);
    IST8310_WriteReg(ist8310, IST8310_REG_CNTL1, IST8310_ODR_200HZ);

    return ist8310;
}

uint8_t IST8310_Acquire(IST8310_Instance *ist8310, IST8310_Data_t *data)
{
    if (!ist8310 || !data) return 1;

    uint8_t buf[6];
    HARD_I2C_Mem_Read(ist8310->i2c, IST8310_REG_DATA_OUT, buf, 6);

    // 解析 XYZ 数据 (小端序)
    int16_t raw[3];
    raw[0] = (int16_t)(buf[1] << 8 | buf[0]);  // X
    raw[1] = (int16_t)(buf[3] << 8 | buf[2]);  // Y
    raw[2] = (int16_t)(buf[5] << 8 | buf[4]);  // Z

    // 转换为 µT
    for (uint8_t i = 0; i < 3; i++) {
        data->mag[i] = (float)raw[i] * IST8310_MAG_SEN;
    }

    return 0;
}
