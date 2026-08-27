#include "daemon.h"

#include <stdlib.h>
#include <string.h>

static Daemon_Instance *daemon_instance[DEVICE_DAEMON_CNT] = {NULL};
static uint8_t idx;

static void Timer_Callback(void 
    #if (RTOS_DAEMON_SUPPORT==1)
    const //在CMSIS_OS1里，需要加入const修饰匹配软件定时器
    #endif
    *argument)
{
    Daemon_Instance *instance = (Daemon_Instance *)argument;

    if(instance->online)
    {
        #if RTOS_DAEMON_SUPPORT
 
        if(instance->daemon_callback !=NULL)
            instance->daemon_callback(instance->device);
        instance->online=0;
        osTimerStop(instance->tim_instance); // 设备已离线，停止守护

        #elif TIM_DAEMON_SUPPORT

        if(instance->count > 0)
            instance->count--;
        else
        {
            if(instance->daemon_callback !=NULL)
                instance->daemon_callback(instance->device);
            instance->online=0;
        }

        #endif
    }
}

#if RTOS_DAEMON_SUPPORT
static void Daemon_TimerInit(Daemon_Instance *instance)
{
    #if (RTOS_DAEMON_SUPPORT==1)
    osTimerDef(Timer, Timer_Callback);
    instance->tim_instance = osTimerCreate(osTimer(Timer), osTimerPeriodic, instance);
    #elif (RTOS_DAEMON_SUPPORT==2)
    instance->tim_instance = osTimerNew(Timer_Callback, osTimerPeriodic, instance, NULL);
    #endif
}
#endif

void Daemon_Reset(Daemon_Instance *instance)
{
    instance->online=1;
    #if RTOS_DAEMON_SUPPORT
    osTimerStart(instance->tim_instance, instance->cycle);
    #elif TIM_DAEMON_SUPPORT
    instance->count=instance->cycle;
    #endif
}

Daemon_Instance *Daemon_Register(Daemon_Init_Config_s *Daemon_config)
{
    Daemon_Instance *instance=(Daemon_Instance*)malloc(sizeof(Daemon_Instance));
    if (instance == NULL) return NULL;
    if (idx >= DEVICE_DAEMON_CNT) {
        free(instance);
        return NULL;
    }
    memset(instance,0,sizeof(Daemon_Instance));

    instance->cycle=Daemon_config->cycle;
    instance->daemon_callback = Daemon_config->daemon_callback;
    instance->device = Daemon_config->device;

    #if RTOS_DAEMON_SUPPORT
    Daemon_TimerInit(instance);
    #elif TIM_DAEMON_SUPPORT
    Daemon_config->device = instance;//将Daemon实例指针传入定时器回调参数
    Daemon_config->tim_config.device = instance;//自己选定时器，为确保可移植性和可解释性，建议用1ms周期的定时器
    Daemon_config->tim_config.tim_callback = Timer_Callback;
    instance->tim_instance = TIM_Register(&Daemon_config->tim_config);
    #endif

    Daemon_Reset(instance);

    daemon_instance[idx++]=instance;
    return instance;
}