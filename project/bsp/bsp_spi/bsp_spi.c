#include "bsp_spi.h"
#include <stdlib.h>
#include <string.h>

#define SPI_MAX_INSTANCES 2

static SPI_Instance *spi_instance[SPI_MAX_INSTANCES];
static uint8_t idx = 0;

SPI_Instance *SPI_Register(SPI_Init_Config_s *config)
{
    if (idx >= SPI_MAX_INSTANCES) return NULL;
    
    SPI_Instance *instance = (SPI_Instance *)malloc(sizeof(SPI_Instance));
    if (!instance) return NULL;
    memset(instance, 0, sizeof(SPI_Instance));
    
    instance->hspi = config->hspi;
    instance->cs_port = config->cs_port;
    instance->cs_pin = config->cs_pin;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);

    spi_instance[idx++] = instance;
    return instance;
}

/**
 * @brief 轮询模式 SPI 传输
 * @note  完全阻塞，使用 HAL_SPI_TransmitReceive 实现
 *         自动拉低/拉高片选，调用前确保 SPI 空闲即可
 */
void SPI_Transfer(SPI_Instance *instance, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
    if (!instance || len == 0) return;
    
    HAL_GPIO_WritePin(instance->cs_port, instance->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(instance->hspi, tx_buf, rx_buf, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(instance->cs_port, instance->cs_pin, GPIO_PIN_SET);
}