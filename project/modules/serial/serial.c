/**
 * @file serial.c/.h
 * @brief serial模块文件
 * @author 乐
 * @date 2026-05-30
 * @version 1.0
 * @brief serial模块文件，提供串口收发功能。作为usart的上层管理，提供不定长数据接收和缓冲区满检测功能。通过注册USART实例和回调函数实现数据接收完成通知。
 * @note 本模块依赖于BSP提供的USART驱动，通过注册USART实例和回调函数实现数据接收完成通知。
 * @note v1.0: 支持多块缓冲区，recv_size决定块数，总缓冲区大小为recv_size × 256。
 */
#include "serial.h"

#include "bsp_usart.h"

#include <string.h>
#include <stdlib.h>

static Serial_Instance *serial_instances[SERIAL_DEVICE_CNT] = {NULL};
static uint8_t serial_idx = 0;

static void Serial_CopyToRecvBuf(Serial_Instance *instance, uint16_t pos)
{
    uint16_t buf_size = instance->recv_size * SERIAL_RXBUFF_LIMIT;
    uint16_t len;

    if (pos > instance->last_pos_parse)
    {
        // 连续：从 last_pos 搬到 pos
        len = pos - instance->last_pos_parse;
        memcpy(instance->recv_buf,
               instance->parse_buf + instance->last_pos_parse, len);
    }
    else
    {
        // 回绕：两段
        uint16_t len1 = buf_size - instance->last_pos_parse;
        uint16_t len2 = pos;
        len = len1 + len2;
        memcpy(instance->recv_buf,
               instance->parse_buf + instance->last_pos_parse, len1);
        memcpy(instance->recv_buf + len1, instance->parse_buf, len2);
    }

    if (instance->rx_callback != NULL)
        instance->rx_callback(len);

    instance->last_pos_parse = pos;
}

static void Serial_FullTimeout(void *device)
{
    Serial_Instance *instance = (Serial_Instance *)device;
    Serial_CopyToRecvBuf(instance, instance->parse_idx * SERIAL_RXBUFF_LIMIT);
}

static void Serial_USART_RX_Callback(void *device, uint8_t recv_pos)
{
    Serial_Instance *instance = (Serial_Instance *)device;

    // 搬运数据到 parse_buf：last_pos_recv 到 recv_pos 复制到 parse_buf 里对应位置
    // 利用 uint8_t 无符号下溢自动处理回绕（如 0-250=6）
    if (instance->recv_size > 1)
    {
        memcpy(instance->parse_buf + SERIAL_RXBUFF_LIMIT * instance->parse_idx + instance->last_pos_recv, instance->usart_instance->recv_buff + instance->last_pos_recv, (uint8_t)(recv_pos - instance->last_pos_recv));
    }

    if (recv_pos % SERIAL_RXBUFF_LIMIT == 0)
    {
        // 全满中断：切换块，喂狗
        if (++instance->parse_idx == instance->recv_size)
            instance->parse_idx = 0;

        Daemon_Reset(instance->full_daemon);
    }
    else
    {
        // 空闲中断：搬运到 recv_buf，关闭全满看门狗
        instance->full_daemon->online = 0;
        Serial_CopyToRecvBuf(instance, instance->parse_idx * SERIAL_RXBUFF_LIMIT + recv_pos);
    }
    instance->last_pos_recv = recv_pos;
}

Serial_Instance *Serial_Register(Serial_Init_Config_s *init_config)
{
    // 超出最大实例数或者空指针，返回NULL
    if (serial_idx >= SERIAL_DEVICE_CNT)
        return NULL;

    if (init_config == NULL || init_config->usart_handle == NULL)
        return NULL;

    // 创建 Serial 实例
    Serial_Instance *instance = (Serial_Instance *)malloc(sizeof(Serial_Instance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(Serial_Instance));

    // 注册 USART 实例
    USART_Init_Config_s usart_config =
    {
        init_config->usart_handle,
        Serial_USART_RX_Callback,
        instance
    };
    instance->usart_instance = USART_Register(&usart_config);
    instance->rx_callback = init_config->rx_callback;

    // 分配双缓冲区
    instance->recv_size = init_config->recv_size;
    if (instance->recv_size <= 1)
        instance->recv_size = 1;

    if (instance->recv_size == 1)
    {
        // 单块模式：parse_buf 直接指向 BSP recv_buff，无需 malloc
        instance->parse_buf = instance->usart_instance->recv_buff;
    }
    else
    {
        instance->parse_buf = malloc(instance->recv_size * SERIAL_RXBUFF_LIMIT);
    }
    instance->recv_buf = malloc(instance->recv_size * SERIAL_RXBUFF_LIMIT);

    if (instance->recv_buf == NULL)
    {
        if (instance->recv_size > 1)
            free(instance->parse_buf);
        free(instance);
        return NULL;
    }
    memset(instance->recv_buf, 0, instance->recv_size * SERIAL_RXBUFF_LIMIT);

    // 注册缓冲区满看门狗
    Daemon_Init_Config_s full_daemon_config =
    {
#if TIM_DAEMON_SUPPORT
        .tim_config = { .htim = init_config->htim },
#endif
        .cycle = 2,
        .device = instance,
        .daemon_callback = Serial_FullTimeout,
    };
    instance->full_daemon = Daemon_Register(&full_daemon_config);

    // 存储到全局数组
    serial_instances[serial_idx] = instance;
    serial_idx++;

    return instance;
}

void Serial_Send(Serial_Instance *instance, uint8_t *send_buf, uint16_t send_size)
{
    if (instance == NULL || instance->usart_instance == NULL)
        return;
    USARTSend(instance->usart_instance, send_buf, send_size);
}
