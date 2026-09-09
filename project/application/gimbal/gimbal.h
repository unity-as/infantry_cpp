/**
 * @file    gimbal.h
 * @brief   云台应用层（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 gimbal 迁移，逻辑不变。收 CMD → 调 gimbal_core 的 class Gimbal API。
 */
#pragma once

#include <stdint.h>

// 软件限位 (deg)
#define GIMBAL_PITCH_MIN  -30.7f
#define GIMBAL_PITCH_MAX   10.5f

struct Gimbal_Cmd {
    float yaw;
    float pitch;
    uint8_t enable;
};

// ECD → 角度换算（8192 = 一圈）
#define GIMBAL_ECD_TO_DEG(ecd)  ((float)(ecd) / 8192.0f * 360.0f)

extern Gimbal_Cmd gimbal_cmd;

void  Gimbal_Init(void);
void  Gimbal_Set_Increment(float yaw_delta, float pitch_delta);
void  Gimbal_Set_Increment_Setpoint(float yaw, float pitch);
void  Gimbal_Set_Mode(uint8_t yaw_mode, uint8_t pitch_mode);
void  Gimbal_Set_Velocity(float yaw_vel, float pitch_vel);
void  Gimbal_AutoAim(float yaw_err, float pitch_err);
void  Gimbal_AutoAim_Reset(void);
void  Gimbal_SyncTarget(void);
float Gimbal_GetYaw(void);
