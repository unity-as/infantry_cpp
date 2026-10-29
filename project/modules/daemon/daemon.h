/**
 * @file    daemon.h
 * @brief   守护进程 / 设备离线看门狗（C → C++）
 * @note    从 C 版 daemon 迁移：struct Daemon_Instance → class Daemon，
 *          Daemon_Register → init、Daemon_Reset → reset，逻辑不变，禁堆。
 *          本项目固定 TIM_DAEMON_SUPPORT=1，故删除 RTOS 分支与死注册表。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）
 */
#pragma once

#include "bsp_tim.h"
#include <stdint.h>

class Daemon {
public:
    /// 初始化配置
    struct Config {
        TIM::Config tim_config;         ///< 时基定时器
        uint32_t cycle;                 ///< 超时 [ms]，按 tim_config 的更新周期换成拍数
        void (*daemon_callback)(void*); ///< 离线回调
        void* device;                   ///< 回调 device
    };

    // —— 实例变量 ——
    uint8_t online_ = 0;   ///< 1=在线，0=离线

private:
    TIM tim_;                                ///< 时基定时器
    uint32_t timeout_ms_ = 0;                ///< 超时 [ms]
    uint32_t cycle_ = 0;                     ///< 超时拍数（由 timeout_ms_ 和定时器周期算出）
    uint32_t count_ = 0;                     ///< 剩余拍数
    void (*daemon_callback_)(void*) = nullptr; ///< 离线回调
    void* device_ = nullptr;                 ///< 回调 device

    // —— static 函数 ——
    static void timerCallback(void* arg);   ///< 替代 Timer_Callback

    // —— 实例函数 ——
    void bindTicks();                       ///< 把 timeout_ms_ 换成当前时基的拍数

public:
    /// 注册/初始化（替代 Daemon_Register，禁堆）
    void init(const Config& config);

    /// 已 init 后补绑时基（可先以 nullptr 注册，再由上层 timbase 统一设置）
    void setTimbase(TIM_HandleTypeDef* htim);

    /// 喂狗（替代 Daemon_Reset）
    void reset();
};
