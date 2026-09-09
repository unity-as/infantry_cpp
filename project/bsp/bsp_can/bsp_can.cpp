/**
 * @file    bsp_can.cpp
 * @brief   CAN 总线实现
 */
#include "bsp_can.h"

#include <string.h>
#include "bsp_dwt.h"

CAN* CAN::instances_[CAN_MX_REGISTER_CNT] = {nullptr};
uint8_t CAN::idx_ = 0;

/// 按句柄建表：惰性 Start、滤波序号、FIFO 轮转（不写死 hcan1/hcan2）
struct BusState {
    CAN_HandleTypeDef* handle = nullptr;
    uint8_t filter_idx = 0;
    uint8_t fifo_sel = 0;
    uint8_t started = 0;
    uint8_t filter_bank_base = 0;  ///< 0 或 CAN_SLAVE_FILTER_BANK_START（F4 共享银行）
};

static BusState buses_[CAN_MX_BUS_CNT];
static uint8_t bus_cnt_ = 0;

static BusState* busStateFor(CAN_HandleTypeDef* h)
{
    if (h == nullptr)
        return nullptr;
    for (uint8_t i = 0; i < bus_cnt_; i++) {
        if (buses_[i].handle == h)
            return &buses_[i];
    }
    if (bus_cnt_ >= CAN_MX_BUS_CNT)
        return nullptr;
    BusState* s = &buses_[bus_cnt_];
    s->handle = h;
    s->filter_idx = 0;
    s->fifo_sel = 0;
    s->started = 0;
    // F4 滤波银行按外设划分（共享寄存器）：CAN1→0–13，CAN2→14–27；不绑 hcan* 全局名
    s->filter_bank_base =
        (h->Instance == CAN1) ? 0 : CAN_SLAVE_FILTER_BANK_START;
    bus_cnt_++;
    return s;
}

static void ensureStarted(CAN_HandleTypeDef* h)
{
    BusState* s = busStateFor(h);
    if (s == nullptr || s->started)
        return;

    HAL_CAN_Start(h);
    HAL_CAN_ActivateNotification(h, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(h, CAN_IT_RX_FIFO1_MSG_PENDING);
    s->started = 1;
}

void CAN::addFilter()
{
    BusState* s = busStateFor(can_handle_);
    if (s == nullptr)
        return;

    CAN_FilterTypeDef can_filter_conf{};

    // 过滤器配置,目前使用最简单的id列表模式,只过滤标准id,不使用掩码
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
    // 使用16位can（32位can会占用两个过滤器）
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
    // 本总线内负载均衡,交替使用两个FIFO
    can_filter_conf.FilterFIFOAssignment = (s->fifo_sel++ % 2) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    can_filter_conf.SlaveStartFilterBank = CAN_SLAVE_FILTER_BANK_START;
    // 标准id在CAN过滤器中占位高5位,因此左移5位
    can_filter_conf.FilterIdLow = static_cast<uint16_t>(rx_id_ << 5);
    can_filter_conf.FilterBank = s->filter_bank_base + s->filter_idx++;
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    HAL_CAN_ConfigFilter(can_handle_, &can_filter_conf);
}

void CAN::init(const Config& config)
{
    if (idx_ >= CAN_MX_REGISTER_CNT)
        return;
    if (config.can_handle == nullptr)
        return;
    for (size_t i = 0; i < idx_; i++) {
        if (instances_[i]->rx_id_ == config.rx_id && instances_[i]->can_handle_ == config.can_handle &&
            config.rx_id != 0)
            return;  // 已注册相同 can 和 id（rx_id 为 0 的是 TX 组，不检查）
    }

    tx_conf_.IDE = CAN_ID_STD;
    tx_conf_.RTR = CAN_RTR_DATA;
    tx_conf_.DLC = 0x08;
    can_handle_ = config.can_handle;
    rx_id_ = config.rx_id;

    ensureStarted(can_handle_);
    // TX 组（rx_id==0）不占滤波银行
    if (rx_id_ != 0)
        addFilter();

    instances_[idx_++] = this;
}

uint8_t CAN::transmit(float timeout)
{
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(can_handle_) == 0)
        if (DWT_GetTimeline_ms() - dwt_start > timeout)
            return 0;
    uint32_t tx_mailbox;
    if (HAL_CAN_AddTxMessage(can_handle_, &tx_conf_, tx_buff_, &tx_mailbox))
        return 0;
    return 1;
}

void CAN::setCallback(Callback callback, void* device)
{
    callback_ = callback;
    device_ = device;
}

void CAN::fifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox)) {
        HAL_CAN_GetRxMessage(hcan, fifox, &rxconf, can_rx_buff);
        for (size_t i = 0; i < idx_; i++) {
            if (hcan == instances_[i]->can_handle_ && rxconf.StdId == instances_[i]->rx_id_) {
                if (instances_[i]->callback_ != nullptr) {
                    instances_[i]->rx_len_ = rxconf.DLC;
                    memcpy(instances_[i]->rx_buff_, can_rx_buff, rxconf.DLC);
                    instances_[i]->callback_(instances_[i]->device_);
                }
                return;
            }
        }
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN::fifoCallback(hcan, CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN::fifoCallback(hcan, CAN_RX_FIFO1);
}
