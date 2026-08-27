#ifndef SERIAL_H
#define SERIAL_H

#include "bsp_usart.h"
#include "daemon.h"
#include "stdint.h"

#define SERIAL_DEVICE_CNT   5
#define SERIAL_RXBUFF_LIMIT 256

typedef struct
{
    USART_Instance *usart_instance;
    Daemon_Instance *full_daemon;
    uint8_t *parse_buf;      // DMA 写入（recv_size × 256）
    uint8_t *recv_buf;       // 给 application 读（recv_size × 256）
    uint8_t recv_size;       // 块数
    uint8_t parse_idx;       // 当前块索引
    uint8_t last_pos_recv;     // BSP recv_buff 内位置（0-255），作为搬运起始点
    uint16_t last_pos_parse;       // parse_buf 内绝对位置
    void (*rx_callback)(uint16_t len);  // 数据接收完成回调
} Serial_Instance;

typedef struct
{
    UART_HandleTypeDef *usart_handle;
#if TIM_DAEMON_SUPPORT
    TIM_HandleTypeDef *htim;
#endif
    uint8_t recv_size;
    void (*rx_callback)(uint16_t len);
} Serial_Init_Config_s;

Serial_Instance *Serial_Register(Serial_Init_Config_s *init_config);
void Serial_Send(Serial_Instance *instance, uint8_t *send_buf, uint16_t send_size);

#if SERIAL_RXBUFF_LIMIT < 1 || SERIAL_RXBUFF_LIMIT > 256
#error "SERIAL_RXBUFF_LIMIT: invalid value, must be between 1 and 256"
#endif

#if SERIAL_RXBUFF_LIMIT != 256
#warning "SERIAL_RXBUFF_LIMIT is not 256, recommend using 256 as single block buffer size"
#endif

#endif
