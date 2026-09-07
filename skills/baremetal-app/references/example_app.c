/**
 * @file    example_app.c
 * @brief   裸机应用层黄金例程（精简提炼版）
 *
 * 模式要点：指针持有、配置集中、回调不决策、轮询消费、外设经模块封装。
 * 基于 ARMOR_PLATE 提炼，模块名/引脚以你的工程为准，抄模式不抄参数。
 */

//app
#include "app.h"
//module
#include "piezo.h"
#include "w2812b.h"
#include "color_sync.h"
//bsp
#include "rtt_debug.h"

/* ──────── 模块实例：app 只持有指针，不持有 bsp 外设实例 ──────── */
static Piezo_Instance    *hit_sensor = NULL;   /* 数据源模块：击打检测 */
static WS2812B_Instance  *led        = NULL;   /* 纯输出模块：灯带     */
static ColorSyncInstance *sync       = NULL;   /* 通信模块：封装 CAN   */

/* ──────── 配置：板级细节全部集中在此（designated initializer） ──────── */
static Piezo_Config_s piezo_cfg = {
    .hadc              = &hadc,
    .channel           = ADC_CHANNEL_0,
    .rank              = 1,
    .sampling_time     = ADC_SAMPLETIME_1CYCLE_5,
};

#define HIT_PEAK_ADC_MIN  680

/* ──────── setup：只做 Register + 初始状态，禁止任何 HAL 初始化 ──────── */
void setup(void)
{
    RTT_DEBUG_INIT();

    hit_sensor = Piezo_Register(&piezo_cfg);
    led        = WS2812B_Register(&htim3, TIM_CHANNEL_1, 6);

    ColorSync_Config_s sync_cfg = {
        .can_handle  = &hcan,
        .local_color = 0,
    };
    sync = ColorSyncRegister(&sync_cfg);   /* 通信经模块封装，app 不碰 bsp */

    WS2812B_SetColor(led, 255, 0, 0);
}

/* ──────── loop：轮询消费标志位，决策在主循环 ──────── */
void loop(void)
{
    Piezo_Process(hit_sensor);              /* 轮询驱动模块，状态在模块内部流转 */
    if (hit_sensor->new_data_ready) {       /* 标志位由中断/回调置起 */
        uint32_t peak = Piezo_GetPeakAdc(hit_sensor);
        Piezo_ClearNewDataFlag(hit_sensor);

        if (peak >= HIT_PEAK_ADC_MIN) {
            WS2812B_SetColor(led, 255, 0, 0);
            ColorSync_Broadcast(sync, 1);   /* app 只调模块 API */
        }
    }
}
