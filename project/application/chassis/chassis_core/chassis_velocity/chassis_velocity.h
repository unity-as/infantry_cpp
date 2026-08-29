/**
 * @file    chassis_velocity.h
 * @brief   底盘四轮速度环 + 功率控制（C → C++）
 * @note    从 C 版 chassis_velocity 迁移：struct ChassisVelocity_Instance → class ChassisVelocity，
 *          ChassisVelocity_Register → init，逻辑不变，禁堆（malloc 实例改为内嵌对象）。
 */
#pragma once

#include <math.h>
#include "dji_motor.h"
#include "pid.h"

#define CHASSIS_WHEEL_NUM 4

// Kt(0.3 N·m/A, 输出轴) × π/180；P(W)=系数×I_q(A)×velocity(输出轴deg/s)
#define CHASSIS_POWER_COEF 0.005236f
#define CHASSIS_GET_POWER(motor)  fmaxf(0.0f, CHASSIS_POWER_COEF * (motor)->current_ * (motor)->velocity_)

class ChassisVelocity {
public:
    /// 初始化配置
    struct Config {
        CAN_HandleTypeDef* can_handle;  // 底盘电机CAN句柄
        uint8_t motor_id_rf;            // 右前轮电机ID
        uint8_t motor_id_lf;            // 左前轮电机ID
        uint8_t motor_id_lr;            // 左后轮电机ID
        uint8_t motor_id_rr;            // 右后轮电机ID
        float reduction_ratio;          // 减速比
        PID::Config pid_velocity;       // 位置式速度环 PID 配置（四轮共用）
        float back_calc_coef;           // 反算增益 ≈1/Ti（=ki/kp），实现时调
    };

    void init(const Config& config);                           // 替代 ChassisVelocity_Register
    void setTarget(const float omega_deg[CHASSIS_WHEEL_NUM]);  // 替代 ChassisVelocity_SetTarget
    void tick();                                               // 替代 ChassisVelocity_Tick
    void setEnable(uint8_t enable);                            // 替代 ChassisVelocity_SetEnable
    void setPowerLimit(float limit);                           // 替代 ChassisVelocity_SetPowerLimit
    float getPower();                                          // 替代 ChassisVelocity_GetPower

private:
    void backCalc(float scale);   // 替代 ChassisVelocity_BackCalc

    DJIMotor motor_[CHASSIS_WHEEL_NUM];        // 四轮电机实例 [rf, lf, lr, rr]
    PID pid_velocity_[CHASSIS_WHEEL_NUM];      // 四轮外置速度环（位置式）
    float target_omega_[CHASSIS_WHEEL_NUM] = {};  // 本周期目标速度 (deg/s, 输出轴)
    float current_raw_[CHASSIS_WHEEL_NUM] = {};   // 本周期速度环输出 I_raw (A)
    float back_calc_coef_ = 0.0f;              // 反算增益
    float power_limit_ = 0.0f;                 // 功率上限 (W)：<=0 不限流
    float power_total_ = 0.0f;                 // 实测总功率 (W)，供监测
    float scale_ = 1.0f;                       // 当前缩放比 k（1=未限流）
    uint8_t enable_ = 0;                       // 使能：0=速度环不跑、PID 复位、输出 0 电流
};
