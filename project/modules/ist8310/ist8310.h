#ifndef IST8310_H
#define IST8310_H

#include "bsp_i2c.h"
#include "bsp_gpio.h"
#include "bsp_dwt.h"
#include "ist8310_reg.h"
#include <stdint.h>

/*---------- 数据结构 ----------*/

typedef struct {
    float mag[3];  // µT (微特斯拉)
} IST8310_Data_t;

/*---------- 实例结构体 ----------*/

typedef struct {
    HARD_I2C_Instance *i2c;
    GPIO_Instance *rst;      // 复位引脚
    uint8_t data_ready;
} IST8310_Instance;

/*---------- 初始化配置 ----------*/

typedef struct {
    HARD_I2C_Config i2c_config;
    GPIO_Init_Config_s rst_config;
} IST8310_Init_Config_s;

/*---------- API ----------*/

IST8310_Instance *IST8310_Register(IST8310_Init_Config_s *config);
uint8_t IST8310_Acquire(IST8310_Instance *ist8310, IST8310_Data_t *data);

#endif
