#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
} HARD_I2C_Instance;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
} HARD_I2C_Config;

#define I2C_ACK    0
#define I2C_NACK   1

HARD_I2C_Instance *HARD_I2C_Init(HARD_I2C_Config *config);
void HARD_I2C_Write(HARD_I2C_Instance *instance, uint8_t *data, uint16_t length);
void HARD_I2C_Read(HARD_I2C_Instance *instance, uint8_t *data, uint16_t length);
void HARD_I2C_Mem_Write(HARD_I2C_Instance *instance, uint8_t reg_addr, uint8_t *data, uint16_t length);
void HARD_I2C_Mem_Read(HARD_I2C_Instance *instance, uint8_t reg_addr, uint8_t *data, uint16_t length);

#endif