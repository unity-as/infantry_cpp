/**
 * @file    chassis.h
 * @brief   底盘模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 chassis 迁移，逻辑不变，禁堆（chassis_inst 由指针改为全局对象）。
 */
#pragma once

#include <stdint.h>
#include "chassis_motion.h"

enum Chassis_Mode {
    CHASSIS_MODE_NO_ROTATION,
    CHASSIS_MODE_FOLLOW,
    CHASSIS_MODE_LITTLE_TOP,
};

struct Chassis_Cmd {
    float v;
    float w_rot;
    float yaw_motor_angle;
    Chassis_Mode mode;
    uint8_t enable;
};

extern Chassis_Cmd chassis_cmd;
extern ChassisMotion chassis_inst;

void Chassis_Init(void);
void Chassis_SetMode(Chassis_Mode mode);
void Chassis_SetPowerLimit(float limit);
float Chassis_GetPower(void);
