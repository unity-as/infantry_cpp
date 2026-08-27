#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "spi.h"   // 根据你的实际型号修改
#include <stdint.h>

typedef struct {
    SPI_HandleTypeDef *hspi;        // SPI 句柄（CubeMX 配置为主机、无 DMA/中断）
    GPIO_TypeDef      *cs_port;     // 软件片选端口
    uint16_t           cs_pin;      // 软件片选引脚
} SPI_Instance;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} SPI_Init_Config_s;

SPI_Instance *SPI_Register(SPI_Init_Config_s *config);
void SPI_Transfer(SPI_Instance *instance, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);

#endif