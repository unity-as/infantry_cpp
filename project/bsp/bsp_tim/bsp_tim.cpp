/**
 * @file    bsp_tim.cpp
 * @brief   TIM 时基中断实现
 */
#include "bsp_tim.h"

namespace {

bool onApb2(TIM_TypeDef* tim) {
    return tim == TIM1 || tim == TIM8 || tim == TIM9 || tim == TIM10 || tim == TIM11;
}

uint32_t kernelClock(TIM_TypeDef* tim) {
    if (onApb2(tim)) {
        uint32_t pclk = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
            pclk *= 2u;
        return pclk;
    }
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        pclk *= 2u;
    return pclk;
}

}  // namespace

TIM* TIM::instances_[TIM_MX_DEVICE_NUM] = {nullptr};
uint8_t TIM::idx_ = 0;

void TIM::init(const Config& config) {
    if (idx_ >= TIM_MX_DEVICE_NUM)
        return;

    htim_ = config.htim;
    instances_[idx_++] = this;

    startIT();
}

void TIM::setHandle(TIM_HandleTypeDef* htim) {
    htim_ = htim;
    startIT();
}

void TIM::startIT() {
    if (htim_ == nullptr)
        return;
    HAL_TIM_Base_Start_IT(htim_);
}

void TIM::stopIT() {
    if (htim_ == nullptr)
        return;
    HAL_TIM_Base_Stop_IT(htim_);
}

void TIM::setCallback(Callback callback, void* device) {
    callback_ = callback;
    device_ = device;
}

uint32_t TIM::updatePeriodUs(TIM_HandleTypeDef* htim) {
    if (htim == nullptr || htim->Instance == nullptr)
        return 0;
    const uint32_t clk = kernelClock(htim->Instance);
    if (clk == 0)
        return 0;
    const uint64_t counts = static_cast<uint64_t>(htim->Init.Prescaler + 1u) *
                            static_cast<uint64_t>(htim->Init.Period + 1u) *
                            static_cast<uint64_t>(htim->Init.RepetitionCounter + 1u);
    return static_cast<uint32_t>((counts * 1000000ull) / clk);
}

void TIM::periodElapsedCallback(TIM_HandleTypeDef* htim) {
    for (uint8_t i = 0; i < idx_; i++) {
        TIM* tim = instances_[i];
        if (tim->callback_ != nullptr && tim->htim_ == htim)
            tim->callback_(tim->device_);
    }
}

extern "C" void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    TIM::periodElapsedCallback(htim);
}
