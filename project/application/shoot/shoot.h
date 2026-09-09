/**
 * @file    shoot.h
 * @brief   射击模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 shoot 迁移，逻辑不变。
 */
#pragma once

#include <stdint.h>

#include "config.h"

struct Shoot_Cmd {
    uint8_t enable;
};

extern Shoot_Cmd shoot_cmd;

#define MAG_DEG_PER_ROUND   (360.0f / MAG_CAPACITY)           // 45°
#define MAG_FEED_SPEED     (MAG_DEG_PER_ROUND * 1000.0f / SHOOT_PERIOD_MS)  // 拨盘转速 (deg/s), 由发弹周期和装载量算出
#define MAG_SPEED_DEGS     MAG_FEED_SPEED   // 拨盘连发/单发速度 (输出轴 deg/s)
#define MAG_MS_PER_ROUND  ((uint16_t)(MAG_DEG_PER_ROUND / MAG_SPEED_DEGS * 1000.0f + 0.5f))  // 63ms

void Shoot_Init(void);
void Shoot_Fire(uint8_t count);
