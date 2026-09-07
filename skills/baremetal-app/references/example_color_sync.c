/**
 * @file    example_color_sync.c
 * @brief   通信模块例程：颜色同步 CAN（演示 app 不裸用 bsp 外设）
 *
 * 模式：模块封装 bsp（CAN），app 只调模块 API。
 */

/* ===== color_sync.h ===== */
/*
#ifndef COLOR_SYNC_H
#define COLOR_SYNC_H

#include <stdint.h>
#include "bsp_can.h"

typedef struct {
    CANInstance *can;        /* bsp 实例在模块内部，app 看不到 */
    uint8_t color;           /* 状态归模块 */
} ColorSyncInstance;

typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint8_t local_color;
} ColorSync_Config_s;

ColorSyncInstance *ColorSyncRegister(ColorSync_Config_s *cfg);
uint8_t ColorSync_GetColor(ColorSyncInstance *sync);
void ColorSync_Broadcast(ColorSyncInstance *sync, uint8_t color);

#endif
*/

#include "color_sync.h"

#include <stdlib.h>
#include <string.h>

/* 回调在模块内部：owner 还原 → 解析 → 不决策 */
static void Sync_Callback(CANInstance *can)
{
    ColorSyncInstance *sync = (ColorSyncInstance *)can->device;
    sync->color = can->rx_buff[0] & 0x01;   /* 解析归模块 */
}

ColorSyncInstance *ColorSyncRegister(ColorSync_Config_s *cfg)
{
    ColorSyncInstance *s = malloc(sizeof(*s));
    if (!s) return NULL;
    memset(s, 0, sizeof(*s));

    CAN_Init_Config_s can_cfg = {
        .can_handle          = cfg->can_handle,
        .rx_id               = 0x300,
        .can_module_callback = Sync_Callback,
        .device              = s,
    };
    s->can   = CANRegister(&can_cfg);      /* 组合子模块 */
    s->color = cfg->local_color;
    return s;
}

uint8_t ColorSync_GetColor(ColorSyncInstance *sync)
{
    return sync ? sync->color : 0;
}

void ColorSync_Broadcast(ColorSyncInstance *sync, uint8_t color)
{
    if (!sync) return;
    sync->can->tx_buff[0] = color;         /* 模块内部操作 bsp 实例 */
    CANTransmit(sync->can, 10);
}
