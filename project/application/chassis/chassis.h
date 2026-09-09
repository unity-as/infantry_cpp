/**
 * @file    chassis.h
 * @brief   底盘应用层（自由函数 + 全局 chassis_cmd）
 * @note    对外只暴露 Chassis_Cmd；core（ChassisMotion）不出现在本头文件。
 */
#pragma once

#include <stdint.h>

enum Chassis_Mode {
    CHASSIS_MODE_NO_ROTATION,
    CHASSIS_MODE_FOLLOW,
    CHASSIS_MODE_LITTLE_TOP,      ///< 小陀螺正向（恒定 +w）
    CHASSIS_MODE_LITTLE_TOP_REV,  ///< 小陀螺反向（恒定 -w）
};

/** 小陀螺恒定角速度 (rad/s)，约 1.5 圈/秒 */
#define CHASSIS_LITTLE_TOP_W  (1.5f * 2.0f * 3.14159f)

struct Chassis_Cmd {
    float v;               // 平移速度 (m/s)
    float theta;           // 平移方向 (rad)，场心系
    float w_rot;           // 手动角速度 (rad/s)，NO_ROTATION 用
    float yaw_motor_angle; // 云台 yaw 角 (deg)，FOLLOW 用
    Chassis_Mode mode;
    uint8_t enable;
};

extern Chassis_Cmd chassis_cmd;

void Chassis_Init(void);
void Chassis_SetMode(Chassis_Mode mode);
void Chassis_SetPowerLimit(float limit);
float Chassis_GetPower(void);
