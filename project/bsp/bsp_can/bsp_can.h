/**
 * @file    bsp_can.h
 * @brief   CAN 总线（C → C++）
 * @note    从 C 版 bsp_can 迁移：struct CAN_Instance → class CAN，CANRegister → init、
 *          CANTransmit → transmit，逻辑不变，禁堆（原 malloc 实例改为栈/全局对象）。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）
 */
#pragma once

#include <stdint.h>
#include "can.h"

#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define CAN_MX_BUS_CNT 2           // 按句柄建表上限（F4 双路；不绑死 hcan1/2 名）
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define CAN_SLAVE_FILTER_BANK_START 14  // F4：CAN2 滤波银行起始（与 Master 共享 0–27）

class CAN {
public:
    using Callback = void (*)(void*);

    /// 初始化配置
    struct Config {
        CAN_HandleTypeDef* can_handle;  ///< CAN 句柄
        uint32_t rx_id;                 ///< 接收 ID（0 表示 TX 组，不加滤波）
    };

    // —— 实例变量 ——
    CAN_HandleTypeDef* can_handle_ = nullptr;  ///< CAN 句柄
    CAN_TxHeaderTypeDef tx_conf_{};            ///< 发送配置
    uint8_t tx_buff_[8]{};                     ///< 发送缓存
    uint8_t rx_buff_[8]{};                     ///< 接收缓存
    uint8_t rx_len_ = 0;                       ///< 接收长度

private:
    uint32_t rx_id_ = 0;                ///< 接收 ID
    Callback callback_ = nullptr;       ///< 接收回调
    void* device_ = nullptr;            ///< 回调 device

    // —— static 变量 ——
    static CAN* instances_[CAN_MX_REGISTER_CNT];    ///< 实例注册表
    static uint8_t idx_;                            ///< 已注册数

    // —— static 函数 ——
public:
    static void fifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifox);  ///< FIFO 中断分发

    // —— 实例函数 ——
    void init(const Config& config);    ///< 替代 CANRegister（禁堆）
    uint8_t transmit(float timeout);    ///< 替代 CANTransmit
    void setCallback(Callback callback, void* device);  ///< 设置接收回调
    void setTxId(uint32_t tx_id) { tx_conf_.StdId = tx_id; }
    uint32_t getTxId() const { return tx_conf_.StdId; }

private:
    void addFilter();                   ///< 添加过滤器
};
