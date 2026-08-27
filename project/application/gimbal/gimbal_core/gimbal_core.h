#ifndef GIMBAL_CORE_H
#define GIMBAL_CORE_H

#include "dji_motor.h"
#include "ahrs.h"
#include "pid.h"

/*============================================
 * 控制模式
 ============================================*/

typedef enum {
    GIMBAL_POSITION_MODE,   // 位置环 → 速度环
    GIMBAL_VELOCITY_MODE,   // 直接速度环
} Gimbal_Mode;

/*============================================
 * 云台单轴数据结构 — 封装 PID/电机/目标/状态
 ============================================*/

typedef struct {
    DJIMotor_Instance *motor;
    PID_Instance      *pid_pos;       // 位置环
    PID_Instance      *pid_vel;       // 速度环
    Gimbal_Mode  mode;                // 控制模式
    float  target;                    // 目标角度 [deg]（位置模式）
    float  vel_target;                // 目标角速度 [deg/s]（速度模式）
    float  actual_angle;             // 实际角度 [deg]
    float  actual_vel;               // 实际角速度 [deg/s]
    float  vel_ff;                   // 速度前馈 [deg/s]
    float  curr_ff;                  // 电流前馈
    uint8_t div_cnt;
    uint8_t pos_freq_div;            // 位置环分频
} Gimbal_Axis;

/*============================================
 * 云台实例 — 双轴 (yaw/pitch)
 ============================================*/

typedef struct {
    AHRS_Instance *ahrs;
    Gimbal_Axis    yaw;
    Gimbal_Axis    pitch;
    float yaw_min, yaw_max;          // 限位 [deg]
    float pitch_min, pitch_max;      // 限位 [deg]
} Gimbal_Instance;

/*============================================
 * 初始化配置
 ============================================*/

typedef struct {
    AHRS_Instance      *ahrs;
    CAN_HandleTypeDef  *can_handle;
    TIM_HandleTypeDef  *htim;
    uint8_t  motor_id_yaw;
    uint8_t  motor_id_pitch;
    float initial_angle_yaw;
    float initial_angle_pitch;
    PID_Init_Config_s pid_yaw_pos;
    PID_Init_Config_s pid_pitch_pos;
    PID_Init_Config_s pid_yaw_vel;
    PID_Init_Config_s pid_pitch_vel;
    uint8_t pos_freq_div_yaw;
    uint8_t pos_freq_div_pitch;
    float yaw_min,   yaw_max;
    float pitch_min, pitch_max;
} Gimbal_Init_Config_s;

/*============================================
 * API
 ============================================*/

Gimbal_Instance *Gimbal_Register(Gimbal_Init_Config_s *config);

void Gimbal_Enabale_Pitch(Gimbal_Instance *gc, uint8_t enable);
void Gimbal_Enabale_Yaw(Gimbal_Instance *gc, uint8_t enable);
void   Gimbal_Enable(Gimbal_Instance *gc, uint8_t enable);

void   Gimbal_SetTarget(Gimbal_Instance *gc, float yaw, float pitch);
void   Gimbal_SetIncrement(Gimbal_Instance *gc, float yaw_delta, float pitch_delta);
void   Gimbal_SetPitchAbsolute(Gimbal_Instance *gc, float angle);
void   Gimbal_SetMode(Gimbal_Instance *gc, Gimbal_Mode yaw_mode, Gimbal_Mode pitch_mode);
void   Gimbal_SetVelocity(Gimbal_Instance *gc, float yaw_vel, float pitch_vel);
void   Gimbal_SetVelocityFF(Gimbal_Instance *gc, float yaw_ff, float pitch_ff);
void   Gimbal_SetCurrentFF(Gimbal_Instance *gc, float yaw_ff, float pitch_ff);
float  Gimbal_GetYawAngle(Gimbal_Instance *gc);

#endif
