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

static_assert(SHOOT_FREQ_HZ > 0.0f && SHOOT_FREQ_HZ <= 1000.0f && 1000.0f / SHOOT_FREQ_HZ <= UINT16_MAX,
              "SHOOT_FREQ_HZ must produce a period between 1 and 65535 ms");

#define MAG_DEG_PER_ROUND   (360.0f / MAG_CAPACITY)             // 每发对应的输出轴角度
#define MAG_SPEED_DEGS      (MAG_DEG_PER_ROUND * SHOOT_FREQ_HZ) // 拨盘输出轴速度 deg/s
#define MAG_MS_PER_ROUND    ((uint16_t)(1000.0f / SHOOT_FREQ_HZ + 0.5f)) // 单发周期 ms，四舍五入

void Shoot_Init(void);
void Shoot_Fire(uint8_t count);
