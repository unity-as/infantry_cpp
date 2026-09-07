#ifndef CONFIG_H
#define CONFIG_H

// main.h
#include "main.h"

// bsp
#include "bsp_tim.h"

// modules
#include "daemon.h"

#ifdef __cplusplus
extern "C" {
#endif

// ======= 调试选项 =======

// 开启任意项，可以单独启动不同的模块，方便调试
#define GIMBAL_INIT_DEBUG 0
#define CHASSIS_INIT_DEBUG 0
#define SHOOT_INIT_DEBUG 0

#if (GIMBAL_INIT_DEBUG || CHASSIS_INIT_DEBUG || SHOOT_INIT_DEBUG)
#define DEBUG_INIT_MODE 1
#else
#define DEBUG_INIT_MODE 0
#endif

#define GIMBAL_INIT !(DEBUG_INIT_MODE && !GIMBAL_INIT_DEBUG)// 云台初始化
#define CHASSIS_INIT !(DEBUG_INIT_MODE && !CHASSIS_INIT_DEBUG) // 底盘初始化
#define SHOOT_INIT    !(DEBUG_INIT_MODE && !SHOOT_INIT_DEBUG)// 射击初始化
// ======= 机器人参数 =======

// 云台电机初始位置 — 编码器值
#define GIMBAL_YAW_ECD      6660 - 30 *360 / 8192
#define GIMBAL_PITCH_ECD    2710

// 云台重力补偿torque
#define GIMBAL_PITCH_CURRENT_FF  -0.150f

// ECD → 角度换算（8192 = 一圈）
#define GIMBAL_ECD_TO_DEG(ecd)  ((float)(ecd) / 8192.0f * 360.0f)


#ifdef __cplusplus
}
#endif

#endif
