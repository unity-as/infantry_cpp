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


// ======= 机器人参数 =======

// 云台电机初始位置 — 编码器值
#define GIMBAL_YAW_ECD      5300
#define GIMBAL_PITCH_ECD    2710

// ECD → 角度换算（8192 = 一圈）
#define GIMBAL_ECD_TO_DEG(ecd)  ((float)(ecd) / 8192.0f * 360.0f)


#ifdef __cplusplus
}
#endif

#endif
