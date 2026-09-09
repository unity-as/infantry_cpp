/**
 * @file    servo.h
 * @brief   舵机（C → C++）
 * @note    从 C 版 servo 迁移：struct Servo_Instance → class Servo，
 *          ServoRegister → init、ServoMove → move，逻辑不变，禁堆。
 *          原 C 的死注册表与未使用的 min_duty/max_duty 宏一并删除。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）
 */
#pragma once

#include "bsp_pwm.h"

class Servo {
public:
    /// 初始化配置
    struct Config {
        PWM::Config servo_pwm_config;   // PWM 配置
        float init_angle;               // 初始角度
    };

    // —— 实例变量 ——
private:
    PWM servo_pwm_;                     // PWM 实例

    // —— static 函数 ——
    static float angleToDuty(float angle);  // 替代 AngleToDuty

    // —— 实例函数 ——
public:
    void init(const Config& config);    // 替代 ServoRegister
    void move(float angle);             // 替代 ServoMove
};
