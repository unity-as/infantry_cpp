#ifndef BSP_USART_H
#define BSP_USART_H

#include "usart.h"
__USART_H__

#define DEVICE_USART_CNT 5     //最大支持串口实例
#define USART_RXBUFF_LIMIT 256 //默认256就好

typedef struct
{
    UART_HandleTypeDef *usart_handle;          // HAL USART 句柄
    uint8_t recv_buff[USART_RXBUFF_LIMIT];     //缓冲区
    void (*module_callback)(void*,uint8_t);//接收完成回调，参数是用户数据指针和接收数据长度
    void *device;                              //回调函数的用户数据指针
} USART_Instance;

//设计为不包含recv_size有两个理由：
//1. 老代码的recv_size是在结构体里本来就没有配置缓冲区动态内存的分配，起不到省空间的作用，反而增加了复杂度和维护成本。
//2. DMA模式下，固定为256字节的缓冲区大小更适合硬件特性，且不需要动态调整recv_size，简化了代码逻辑和使用方式。

typedef struct
{
    UART_HandleTypeDef *usart_handle;       // HAL USART 句柄
    void (*module_callback)(void*,uint8_t);  //接收完成回调，参数是用户数据指针和接收数据长度
    void *device;                              //回调函数的用户数据指针
} USART_Init_Config_s;

USART_Instance *USART_Register(USART_Init_Config_s *init_config);                     // 注册 USART 实例，返回实例指针，失败返回 NULL
void USARTSend(USART_Instance *_instance, uint8_t *send_buf, uint16_t send_size);     // 发送数据，默认了DMA+Circular方式发送
void USART_ServiceInit(USART_Instance *_instance);                                    // USART 服务初始化，设置接收缓冲区和启动接收

#if USART_RXBUFF_LIMIT > 256 || USART_RXBUFF_LIMIT < 1
#error "USART_RXBUFF_LIMIT must be between 1 and 256"
#endif

#endif