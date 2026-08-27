/**
 * @file bsp_usart.c/.h
 * @brief USART底层驱动文件
 * @author 乐ge
 * @date 2026-05-30
 * @version 1.0
 * @brief USART底层驱动文件，非入侵设计，提供USART的注册、发送和接收服务。支持多实例管理，DMA+Circular全双工模式+不定长接收。固定256字节接收长度，避免非必要的提高复杂度的设计。
 * @note 初始版本，仅提供全满和空闲中断，并要求配好DMA。记住：全满中断触发会清除空闲标志位，串口的单次发送长度受限建议搭配无锁队列，建议配合mudules的上层双缓冲串口管理使用。
 */

#include "bsp_usart.h"

#include <stdlib.h>
#include <string.h>

static uint8_t idx;
static USART_Instance *usart_instance[DEVICE_USART_CNT] = {NULL};

void USART_ServiceInit(USART_Instance *_instance)// USART 服务初始化，设置接收缓冲区和启动接收中断
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, USART_RXBUFF_LIMIT);
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USART_Instance *USART_Register(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT)
        return NULL;

    for (uint8_t i = 0; i < idx; i++)
        if (usart_instance[i]->usart_handle == init_config->usart_handle)// 判断是否已经注册过该 USART 实例，避免重复注册
            return NULL;

    USART_Instance *instance = (USART_Instance *)malloc(sizeof(USART_Instance));// 分配内存空间，并清零
    memset(instance, 0, sizeof(USART_Instance));

    // 初始化实例结构体成员变量
    instance->usart_handle = init_config->usart_handle;
    instance->module_callback = init_config->module_callback;
    instance->device = init_config->device;

    //注册 USART 实例，并返回指针
    usart_instance[idx++] = instance;

    //初始化 USART 服务，并启动接收中断
    USART_ServiceInit(instance);

    return instance;
}

void USARTSend(USART_Instance *_instance, uint8_t *send_buf, uint16_t send_size)// 发送数据，默认了DMA方式发送
{
    HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; i++)
    { 
        USART_Instance *instance = usart_instance[i];
        if (huart == instance->usart_handle)
        {
            // 计算当前接收到的数据长度
            instance->module_callback(instance->device, (uint8_t)Size);

            USART_ServiceInit(instance);// 重新启动接收中断
            return;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; i++)
    { 
        if (huart == usart_instance[i]->usart_handle)
        {
            USART_ServiceInit(usart_instance[i]);//发生错误，重新启动接收中断，先就这样吧，反正也没啥好做的
            return;
        }
    }
}