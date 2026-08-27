#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多

/* CAN实例结构体 */
typedef struct
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint8_t tx_buff[8];            // 发送缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8

    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(void*); // callback needs an instance to tell among registered ones
    void *device;                       // 使用can外设的模块指针(即device指向的模块拥有此can实例,是父子关系)
} CAN_Instance;

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(void*); // 处理接收数据的回调函数
    void *device;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

CAN_Instance *CANRegister(CAN_Init_Config_s *config);
uint8_t CANTransmit(CAN_Instance *_instance, float timeout);

#endif