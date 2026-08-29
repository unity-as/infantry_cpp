/**
 * @file    cmd.h
 * @brief   指令/主控模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 cmd 迁移，逻辑不变。cmd_ahrs 由指针改为全局对象。
 */
#pragma once

#include "ahrs.h"

// ============ 遥控器 → 云台（增量式：满偏 ±660，1kHz 调用）============
// 满偏转速 deg/s（可调），scale = 满偏转速 / 满偏量(660) / 1000(ms→s)
#define REMOTE_GIMBAL_YAW_MAX_DEG_S     200.0f
#define REMOTE_GIMBAL_PITCH_MAX_DEG_S    100.0f
#define REMOTE_GIMBAL_YAW_SCALE    (REMOTE_GIMBAL_YAW_MAX_DEG_S / (660.0f * 1000.0f))
#define REMOTE_GIMBAL_PITCH_SCALE  (REMOTE_GIMBAL_PITCH_MAX_DEG_S / (660.0f * 1000.0f))

// ============ 遥控器 → 底盘（满偏 ±660）============
#define REMOTE_CHASSIS_V_MAX_MPS        1.5f   // 满偏移速 m/s（可调）
#define REMOTE_CHASSIS_V_SCALE         (REMOTE_CHASSIS_V_MAX_MPS / 660.0f)   // m/s per unit
// 满偏角速度：拨轮满量程(±660) → ±REMOTE_CHASSIS_W_MAX_REV 圈/秒
#define REMOTE_CHASSIS_W_MAX_REV        1.5f    // 圈/秒（可调）
#define REMOTE_CHASSIS_W_SCALE         (REMOTE_CHASSIS_W_MAX_REV * 2.0f * 3.14159f / 660.0f)   // rad/s per unit

// ============ 键鼠模式 ============
#define KEYBOARD_CHASSIS_V_MPS          1.5f    // 键盘底盘移速 m/s（可调）
// 鼠标 → 云台增量（增量式，不好界定满偏，保持原样）
#define MOUSE_GIMBAL_YAW_SCALE          0.001f
#define MOUSE_GIMBAL_PITCH_SCALE        0.0005f

extern AHRS cmd_ahrs;

void Cmd_Init(void);
void Cmd_Task(void);
