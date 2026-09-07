/**
 * @file    gimbal_core.h
 * @brief   云台双轴串级 PID 控制核心（C → C++）
 * @note    从 C 版 gimbal_core 迁移：struct Gimbal_Instance → class Gimbal，
 *          Gimbal_Register → init，逻辑不变，禁堆（malloc 实例改为内嵌对象）。
 */
#pragma once

#include "dji_motor.h"
#include "ahrs.h"
#include "pid.h"
#include "bsp_tim.h"

#define GIMBAL_MAX  4

class Gimbal {
public:
    /// 控制模式
    enum class Mode : uint8_t {
        Position = 0,   // 位置环 → 速度环
        Velocity,       // 直接速度环
    };

    /// 云台单轴数据结构 — 封装 PID/电机/目标/状态
    struct Axis {
        DJIMotor motor;               // 电机（内嵌，禁堆）
        PID pid_pos;                  // 位置环
        PID pid_vel;                  // 速度环
        Mode mode = Mode::Position;   // 控制模式
        float target = 0.0f;          // 目标角度 [deg]（位置模式）
        float vel_target = 0.0f;      // 目标角速度 [deg/s]（速度模式）
        float actual_angle = 0.0f;    // 实际角度 [deg]
        float actual_vel = 0.0f;      // 实际角速度 [deg/s]
        float vel_ff = 0.0f;          // 速度前馈 [deg/s]
        float curr_ff = 0.0f;         // 电流前馈
        uint8_t div_cnt = 0;
        uint8_t pos_freq_div = 1;     // 位置环分频
    };

    /// 初始化配置
    struct Config {
        AHRS* ahrs;
        CAN_HandleTypeDef* can_handle;
        TIM_HandleTypeDef* htim;
        uint8_t motor_id_yaw;
        uint8_t motor_id_pitch;
        float initial_angle_yaw;
        float initial_angle_pitch;
        PID::Config pid_yaw_pos;
        PID::Config pid_pitch_pos;
        PID::Config pid_yaw_vel;
        PID::Config pid_pitch_vel;
        uint8_t pos_freq_div_yaw;
        uint8_t pos_freq_div_pitch;
        float yaw_min, yaw_max;
        float pitch_min, pitch_max;
    };

    void init(const Config& config);   // 替代 Gimbal_Register

    // —— 跨模块读取的状态 ——
    AHRS* ahrs_;               // AHRS（gimbal 层 SyncTarget 读其输出）
    Axis yaw_;                 // yaw 轴
    Axis pitch_;               // pitch 轴
    float yaw_min_, yaw_max_;       // 限位 [deg]
    float pitch_min_, pitch_max_;   // 限位 [deg]

    // —— API ——
    void enablePitch(uint8_t enable);                          // 替代 Gimbal_Enabale_Pitch
    void enableYaw(uint8_t enable);                            // 替代 Gimbal_Enabale_Yaw
    void enable(uint8_t enable);                               // 替代 Gimbal_Enable
    void setTarget(float yaw, float pitch);                    // 替代 Gimbal_SetTarget
    void setIncrement(float yaw_delta, float pitch_delta);     // 替代 Gimbal_SetIncrement
    void setPitchAbsolute(float angle);                        // 替代 Gimbal_SetPitchAbsolute
    void setMode(Mode yaw_mode, Mode pitch_mode);              // 替代 Gimbal_SetMode
    void setVelocity(float yaw_vel, float pitch_vel);          // 替代 Gimbal_SetVelocity
    void setVelocityFF(float yaw_ff, float pitch_ff);          // 替代 Gimbal_SetVelocityFF
    void setCurrentFF(float yaw_ff, float pitch_ff);           // 替代 Gimbal_SetCurrentFF
    float getYawAngle();                                       // 替代 Gimbal_GetYawAngle

private:
    void axisUpdate(Axis& axis);               // 替代 Gimbal_AxisUpdate
    void update();                             // 替代 Gimbal_Update
    static void timCallback(void* device);     // 替代 Gimbal_TimHandler

    static Gimbal* instances_[GIMBAL_MAX];     // 实例注册表
    static uint8_t idx_;                       // 已注册数
    static TIM tim_;                           // 更新定时器
};
