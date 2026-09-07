/**
 * @file    bsp_usart.cpp
 * @brief   USART 串口实现
 */
#include "bsp_usart.h"

USART* USART::instances_[DEVICE_USART_CNT] = {nullptr};
uint8_t USART::idx_ = 0;

void USART::serviceInit() {
    HAL_UARTEx_ReceiveToIdle_DMA(usart_handle_, recv_buff_, USART_RXBUFF_LIMIT);
    __HAL_DMA_DISABLE_IT(usart_handle_->hdmarx, DMA_IT_HT);
}

void USART::init(const Config& config) {
    if (idx_ >= DEVICE_USART_CNT)
        return;

    for (uint8_t i = 0; i < idx_; i++)
        if (instances_[i]->usart_handle_ == config.usart_handle)  // 判断是否已经注册过该 USART 实例，避免重复注册
            return;

    usart_handle_ = config.usart_handle;
    instances_[idx_++] = this;

    serviceInit();  // 初始化 USART 服务，并启动接收中断
}

void USART::send(uint8_t* send_buf, uint16_t send_size) {
    HAL_UART_Transmit_DMA(usart_handle_, send_buf, send_size);
}

void USART::setCallback(Callback callback, void* device) {
    callback_ = callback;
    device_ = device;
}

void USART::rxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    for (uint8_t i = 0; i < idx_; i++) {
        USART* instance = instances_[i];
        if (huart == instance->usart_handle_) {
            // 计算当前接收到的数据长度
            instance->callback_(instance->device_, (uint8_t)size);

            instance->serviceInit();  // 重新启动接收中断
            return;
        }
    }
}

void USART::errorCallback(UART_HandleTypeDef* huart) {
    for (uint8_t i = 0; i < idx_; i++) {
        if (huart == instances_[i]->usart_handle_) {
            instances_[i]->serviceInit();  // 发生错误，重新启动接收中断，先就这样吧
            return;
        }
    }
}

// USART 接收/错误中断回调函数（HAL 弱函数覆盖，C 链接）
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    USART::rxEventCallback(huart, Size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    USART::errorCallback(huart);
}
