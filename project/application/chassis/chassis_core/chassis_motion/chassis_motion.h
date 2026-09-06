/**
 * @file    chassis_motion.h
 * @brief   底盘运动学解算（C → C++）
 * @note    从 C 版 chassis_motion 迁移：struct ChassisMotion_Instance → class ChassisMotion，
 *          ChassisMotion_Register → init，逻辑不变，禁堆（malloc 实例改为内嵌对象）。
 */
#pragma once

#include "bsp_tim.h"
#include "chassis_velocity.h"

class ChassisMotion {
public:
    /// 底盘类型
    enum class Type : uint8_t {
        Mecanum = 0,   // 麦克纳姆底盘
        Omni,          // 全向轮底盘
    };

    /// 初始化配置
    struct Config {
        Type type;
        float r;    // 轮半径
        float D;    // 底盘上的轮心间距
        float L;    // 轮子纵向轴距
        float W;    // 轮子横向间距

        ChassisVelocity::Config velocity;  // [2] 速度环+功率（四轮电机）
        TIM_HandleTypeDef* tim_handle;     // 1kHz 定时器（注册先于 dji_motor）
    };

    void init(const Config& config);   // 替代 ChassisMotion_Register

    // —— 底盘输入量 ——
    float theta_ = 0.0f;   // 底盘朝向角度 (rad)
    float v_ = 0.0f;       // 移动速度 (m/s)
    float w_rot_ = 0.0f;   // 旋转速度 (rad/s)

    // —— 下层模块 ——
    ChassisVelocity velocity_;   // [2] 速度环+功率

private:
    void update();                          // 替代 ChassisMotion_Update
    static void timCallback(void* device);  // 替代 ChassisMotion_TimHandler

    /* 运动学参数 */
    float move_scale_ = 0.0f;    // 移动缩放因子
    float rotate_scale_ = 0.0f;  // 旋转缩放因子

    TIM tim_;   // 1kHz 定时器
};
