#ifndef DWT_PROTECT_H
#define DWT_PROTECT_H

/* ========== 平台检测 ========== */
#if __has_include("cmsis_os2.h")
    #include "cmsis_os2.h"
    #define DWT_DAEMON_SUPPORT_RTOS 2
#elif __has_include("cmsis_os.h")
    #include "cmsis_os.h"
    #define DWT_DAEMON_SUPPORT_RTOS 1
#elif __has_include("bsp_tim.h")
    #include "bsp_tim.h"
    #define DWT_DAEMON_SUPPORT_BSP_TIM 1
#else
    #error "DWT Daemon requires CMSIS-RTOS or bsp_tim support"
#endif

/* ========== 配置 ========== */
#define DWT_DAEMON_PERIOD_MS  10000   // 更新时间间隔，单位 ms

/* ========== API ========== */
void DWT_DaemonInit(void);

#endif