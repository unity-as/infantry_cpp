#include "bsp_i2c.h"
#include <stdlib.h>
#include <string.h>

#define HARD_I2C_MAX_INSTANCES 10

static HARD_I2C_Instance *hard_i2c_instances[HARD_I2C_MAX_INSTANCES];
static uint8_t instance_count = 0;

// 初始化硬件I2C实例，动态分配内存并注册到全局实例数组
HARD_I2C_Instance *HARD_I2C_Init(HARD_I2C_Config *config)
{
    if (instance_count >= HARD_I2C_MAX_INSTANCES)
        return NULL;

    HARD_I2C_Instance *instance = (HARD_I2C_Instance *)malloc(sizeof(HARD_I2C_Instance));
    if (instance == NULL) {
        return NULL;
    }
    
    memset(instance, 0, sizeof(HARD_I2C_Instance));
    instance->hi2c = config->hi2c;       // 保存I2C句柄
    instance->dev_addr = config->dev_addr; // 保存设备地址
    
    hard_i2c_instances[instance_count++] = instance; // 注册到全局数组
    
    return instance;
}

void HARD_I2C_Write(HARD_I2C_Instance *instance, uint8_t *data, uint16_t length)
{
    if (instance == NULL || instance->hi2c == NULL) {
        return;
    }
    
    HAL_I2C_Master_Transmit(instance->hi2c, 
                           instance->dev_addr, 
                           data, 
                           length, 
                           100);
}

void HARD_I2C_Read(HARD_I2C_Instance *instance, uint8_t *data, uint16_t length)
{
    if (instance == NULL || instance->hi2c == NULL) {
        return;
    }
    
    HAL_I2C_Master_Receive(instance->hi2c, 
                          instance->dev_addr, 
                          data, 
                          length, 
                          100);
}

void HARD_I2C_Mem_Write(HARD_I2C_Instance *instance, uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    if (instance == NULL || instance->hi2c == NULL) {
        return;
    }
    
    HAL_I2C_Mem_Write(instance->hi2c,
                     instance->dev_addr,
                     reg_addr,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     length,
                     100);
}

void HARD_I2C_Mem_Read(HARD_I2C_Instance *instance, uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    if (instance == NULL || instance->hi2c == NULL) {
        return;
    }
    
    HAL_I2C_Mem_Read(instance->hi2c,
                    instance->dev_addr,
                    reg_addr,
                    I2C_MEMADD_SIZE_8BIT,
                    data,
                    length,
                    100);
}