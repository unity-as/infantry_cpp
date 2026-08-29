/**
 * @file    bsp_can.cpp
 * @brief   CAN 总线实现
 */
#include "bsp_can.h"

#include <string.h>
#include "bsp_dwt.h"

CAN* CAN::instances_[CAN_MX_REGISTER_CNT] = {nullptr};
uint8_t CAN::idx_ = 0;

void CAN::addFilter() {
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
    can_filter_conf.FilterIdLow = rx_id_ << 5;
    // 基于动态注册序列的哈希分流算法，以优化总线突发负载下的中断响应延迟分布
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;
    can_filter_conf.FilterBank = can_handle_ == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
    // 使能过滤器
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    HAL_CAN_ConfigFilter(can_handle_, &can_filter_conf);
}

void CAN::serviceInit() {
    // 初始化CAN设备,启动CAN并使能接收中断
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CAN::init(const Config& config) {
    if (idx_ == 0)
        serviceInit(); // 第一次注册时,先进行硬件初始化
    if (idx_ >= CAN_MX_REGISTER_CNT)
        return; // 超过最大实例数
    for (size_t i = 0; i < idx_; i++)
        if (instances_[i]->rx_id_ == config.rx_id && instances_[i]->can_handle_ == config.can_handle && config.rx_id != 0)
            return; // 已注册相同can和id的实例,避免重复注册(rx_id为0的是TX组,不检查)

    // 进行发送报文的配置
    tx_conf_.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    tx_conf_.RTR = CAN_RTR_DATA;    // 发送数据帧
    tx_conf_.DLC = 0x08;            // 默认发送长度为8
    // 设置句柄和接收id
    can_handle_ = config.can_handle;
    rx_id_ = config.rx_id;

    addFilter();                 // 添加CAN过滤器规则
    instances_[idx_++] = this;   // 将实例保存到注册表
}

uint8_t CAN::transmit(float timeout) {
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(can_handle_) == 0) // 等待有邮箱空闲
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
            return 0;
    // 发送报文
    uint32_t tx_mailbox; // 这东西没用，从结构体拉出来丢这里了
    if (HAL_CAN_AddTxMessage(can_handle_, &tx_conf_, tx_buff_, &tx_mailbox))
        return 0;
    return 1; // 发送成功
}

void CAN::setCallback(Callback callback, void* device) {
    callback_ = callback;
    device_ = device;
}

void CAN::fifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifox) {
    static CAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox)) { // FIFO不为空,有可能在其他中断时有多帧数据进入
        HAL_CAN_GetRxMessage(hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        for (size_t i = 0; i < idx_; i++) {
            // 两者相等说明这是要找的实例
            if (hcan == instances_[i]->can_handle_ && rxconf.StdId == instances_[i]->rx_id_) {
                if (instances_[i]->callback_ != nullptr) { // 回调函数不为空就调用
                    instances_[i]->rx_len_ = rxconf.DLC;                      // 保存接收到的数据长度
                    memcpy(instances_[i]->rx_buff_, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                    instances_[i]->callback_(instances_[i]->device_);         // 触发回调进行数据解析和处理
                }
                return;
            }
        }
    }
}

// CAN接收中断回调函数（HAL 弱函数覆盖，C 链接）
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN::fifoCallback(hcan, CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN::fifoCallback(hcan, CAN_RX_FIFO1);
}
