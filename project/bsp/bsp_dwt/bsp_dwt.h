#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern uint8_t DWT_Init_flag;

//函数占用时间检测宏
#define TIME_ELAPSE(dt, code)                    \
    do                                           \
    {                                            \
        float tstart = DWT_GetTimeline_s();      \
        code;                                    \
        dt = DWT_GetTimeline_s() - tstart;       \
    } while (0)

void DWT_Init();//DWT初始化

float DWT_GetDeltaT(uint32_t *cnt_last);//获取时间差
double DWT_GetDeltaT64(uint32_t *cnt_last);//获取精确时间差

float DWT_GetTimeline_s(void);//获取当前秒
float DWT_GetTimeline_ms(void);//获取当前毫秒
uint64_t DWT_GetTimeline_us(void);//获取当前微秒

void DWT_Delay(float Delay);//DWT延时
void DWT_Delay_Tick(uint64_t delay);
void DWT_Delay_us(float delay);
void DWT_Delay_ms(float delay);

void DWT_SysTimeUpdate(void);//DWT更新

void DWT_TaskInit();//DWT更新任务初始化

#ifdef __cplusplus
}
#endif

#endif /* BSP_DWT_H_ */
