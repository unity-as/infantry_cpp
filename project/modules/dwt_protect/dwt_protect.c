#include "dwt_protect.h"
#include "bsp_dwt.h"

/* ========== RTOS 方案（CMSIS v1 / v2） ========== */
#if DWT_DAEMON_SUPPORT_RTOS

static osTimerId_t dwt_timer = NULL;

static void DWT_DaemonCallback(void *argument)
{
    (void)argument;
    DWT_SysTimeUpdate();
    // 单次定时器，每次超时后重新启动
    if (dwt_timer != NULL)
        osTimerStart(dwt_timer, DWT_DAEMON_PERIOD_MS);
}

void DWT_DaemonInit()
{
    #if (DWT_DAEMON_SUPPORT_RTOS == 1) // CMSIS v1
    osTimerDef(dwt_daemon_timer, DWT_DaemonCallback);
    dwt_timer = osTimerCreate(osTimer(dwt_daemon_timer), osTimerOnce, NULL);
    #elif (DWT_DAEMON_SUPPORT_RTOS == 2) // CMSIS v2
    dwt_timer = osTimerNew(DWT_DaemonCallback, osTimerOnce, NULL, NULL);
    #endif

    if (dwt_timer != NULL)
        osTimerStart(dwt_timer, DWT_DAEMON_PERIOD_MS);
}

/* ========== 裸机方案（bsp_tim） ========== */
#elif DWT_DAEMON_SUPPORT_BSP_TIM

static TIM_Instance *dwt_tim = NULL;
static uint32_t cnt;
static TIM_Init_Config_s tim_config ={
    //裸机此处选取定时器
};

static void DWT_TimCallback(void *device)
{
    if(++cnt == DWT_DAEMON_PERIOD_MS)
    {
        DWT_SysTimeUpdate();
        cnt = 0;
    }
    // 硬件定时器若为周期模式会自动重载
}

void DWT_DaemonInit()
{
    dwt_tim = TIM_Register(&tim_config);
    dwt_tim->tim_callback = DWT_TimCallback;
}

#endif