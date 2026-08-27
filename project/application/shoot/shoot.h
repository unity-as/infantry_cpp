#ifndef SHOOT_H
#define SHOOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
    uint8_t enable;
} Shoot_Cmd;

extern Shoot_Cmd shoot_cmd;

#define FRIC_SPEED_DEGS       1500.0f   // 摩擦轮工作速度 (输出轴 deg/s)

#define MAG_CAPACITY             8       // 拨盘装载量 发/圈
#define MAG_DEG_PER_ROUND   (360.0f / MAG_CAPACITY)           // 45°
#define MAG_SPEED_DEGS     MAG_FEED_SPEED   // 拨盘连发/单发速度 (输出轴 deg/s)
#define MAG_MS_PER_ROUND  ((uint16_t)(MAG_DEG_PER_ROUND / MAG_SPEED_DEGS * 1000.0f + 0.5f))  // 63ms

#define SHOOT_PERIOD_MS           64     // 发弹周期 (ms)
#define MAG_FEED_SPEED     (MAG_DEG_PER_ROUND * 1000.0f / SHOOT_PERIOD_MS)  // 拨盘转速 (deg/s), 由发弹周期和装载量算出

void Shoot_Init(void);
void Shoot_Fire(uint8_t count);

#ifdef __cplusplus
}
#endif

#endif
