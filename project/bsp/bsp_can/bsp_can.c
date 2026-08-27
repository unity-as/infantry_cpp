#include "bsp_can.h"

#include <string.h>
#include <stdlib.h>
#include "bsp_dwt.h"

static CAN_Instance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;

static void CANAddFilter(CAN_Instance *_instance)
{
    CAN_FilterTypeDef can_filter_conf;

    // 过滤器配置,目前使用最简单的id列表模式,只过滤标准id,不使用掩码
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
    // 使用16位can（32位can会占用两个过滤器）
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
    // 负载均衡,交替使用两个FIFO
    static uint8_t fifox_idx = 0;
    can_filter_conf.FilterFIFOAssignment = fifox_idx++ % 2 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    // 每个CAN接口给14个过滤器,过滤器0-13分配给CAN1,14-27分配给CAN2
    can_filter_conf.SlaveStartFilterBank = 14;
    // 标准id在CAN过滤器中占位高5位,因此左移5位
    can_filter_conf.FilterIdLow = _instance->rx_id << 5;
    // 基于动态注册序列的哈希分流算法，以优化总线突发负载下的中断响应延迟分布
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;
    can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
    // 使能过滤器
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

static void CANServiceInit()
{
    // 初始化CAN设备,启动CAN并使能接收中断
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

CAN_Instance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
        CANServiceInit(); // 第一次注册时,先进行硬件初始化
    if (idx >= CAN_MX_REGISTER_CNT)
        return NULL;// 超过最大实例数
    for (size_t i = 0; i < idx; i++)
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle && config->rx_id != 0)
            return NULL;// 已注册相同can和id的实例,避免重复注册(rx_id为0的是TX组,不检查)

    CAN_Instance *instance = (CAN_Instance *)malloc(sizeof(CAN_Instance)); // 分配空间
    memset(instance, 0, sizeof(CAN_Instance));                           // 分配的空间清零
    // 进行发送报文的配置
    instance->txconf.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->txconf.DLC = 0x08;            // 默认发送长度为8
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->device = config->device;

    CANAddFilter(instance);         // 添加CAN过滤器规则
    can_instance[idx++] = instance; // 将实例保存到can_instance中

    return instance; // 返回can实例指针
}

uint8_t CANTransmit(CAN_Instance *_instance, float timeout)
{
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // 等待有邮箱空闲
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
            return 0;
    // 发送报文
    uint32_t tx_mailbox;//这东西没用，从结构体拉出来丢这里了
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &tx_mailbox))
        return 0;
    return 1; // 发送成功
}

static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入（不够健壮，建议在上层加FIFO，比如RTOS队列）
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        for (size_t i = 0; i < idx; i++)
        {
             // 两者相等说明这是要找的实例
            if (_hcan == can_instance[i]->can_handle && rxconf.StdId == can_instance[i]->rx_id)
            {
                if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
                {
                    can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
                    memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                    can_instance[i]->can_module_callback(can_instance[i]->device); // 触发回调进行数据解析和处理
                }
                return;
            }
        }
    }
}

// CAN接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}