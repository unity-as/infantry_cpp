#ifndef BPS_SOFT_I2C_H
#define BPS_SOFT_I2C_H

#include "gpio.h"
__GPIO_H__

/**
 * @brief 软件I2C实例结构体
 * 
 * 用于存储软件I2C实例的信息
 */
typedef struct
{
    GPIO_TypeDef *port;     /*!< GPIO端口 */
    uint32_t i2c_tick;      /*!< I2C时钟节拍 */
    uint16_t scl;           /*!< SCL引脚 */
    uint16_t sda;           /*!< SDA引脚 */
    uint8_t dev_addr;       /*!< 设备地址 */
}SOFT_I2C_Instance;

/**
 * @brief 软件I2C配置结构体
 * 
 * 用于存储软件I2C的配置信息
 */
typedef struct
{
    GPIO_TypeDef *port;     /*!< GPIO端口 */
    uint32_t i2c_tick;      /*!< I2C时钟节拍 */
    uint16_t scl;           /*!< SCL引脚 */
    uint16_t sda;           /*!< SDA引脚 */
    uint8_t dev_addr;       /*!< 设备地址 */
}SOFT_I2C_Config;

#define SOFT_I2C_ACK    0   /*!< 应答信号 */
#define SOFT_I2C_NACK   1   /*!< 非应答信号 */

/**
 * @brief 初始化软件I2C
 * 
 * @param config 配置结构体指针
 * @return SOFT_I2C_Instance* 实例指针
 */
SOFT_I2C_Instance *SOFT_I2C_Init(SOFT_I2C_Config *config);

/**
 * @brief 软件I2C读写
 * 
 * @param instance 实例指针
 * @param reg_addr 寄存器地址
 * @param data 数据指针
 * @param length 数据长度
 */
void SOFT_I2C_Write(SOFT_I2C_Instance *instance, uint8_t *data, uint16_t length);
void SOFT_I2C_Read(SOFT_I2C_Instance *instance, uint8_t *data, uint16_t length);
void SOFT_I2C_Mem_Write(SOFT_I2C_Instance *instance, uint8_t reg_addr,uint8_t *data, uint16_t length);
void SOFT_I2C_Mem_Read(SOFT_I2C_Instance *instance, uint8_t reg_addr,uint8_t *data, uint16_t length);

#endif