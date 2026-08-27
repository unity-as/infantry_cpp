#ifndef DAEMON_H
#define DAEMON_H

//此处做自定义
#define TIM_DAEMON_SUPPORT 1
//

#if (!defined(RTOS_DAEMON_SUPPORT) && !defined(TIM_DAEMON_SUPPORT))

#if __has_include("cmsis_os2.h")// 如果检测到cmsis2_os头文件，则使用cmsis2的定时器功能实现Daemon
    #include "cmsis_os2.h"
    #define RTOS_DAEMON_SUPPORT 2
#elif __has_include("cmsis_os.h")// 如果检测到cmsis_os头文件，则使用cmsis的定时器功能实现Daemon
    #include "cmsis_os.h"
    #define RTOS_DAEMON_SUPPORT 1
#elif __has_include("bsp_tim.h")//若采用的是裸机开发的BSP_TIM，则使用BSP_TIM的定时器功能实现Daemon
    #include "bsp_tim.h"
    #define TIM_DAEMON_SUPPORT 1
#else
    #error "Daemon module requires cmsis or bsp_tim support"
#endif

#endif

#if (RTOS_DAEMON_SUPPORT == 2)
#include "cmsis_os2.h"
#elif (RTOS_DAEMON_SUPPORT == 1)
#include "cmsis_os.h"
#elif (TIM_DAEMON_SUPPORT == 1)
#include "bsp_tim.h"
#endif

#define DEVICE_DAEMON_CNT 20

typedef struct
{
    #if TIM_DAEMON_SUPPORT
    TIM_Instance *tim_instance;
    uint32_t count;
    #endif

    #if (RTOS_DAEMON_SUPPORT == 1)
    osTimerId tim_instance;
    #elif (RTOS_DAEMON_SUPPORT == 2)
    osTimerId_t tim_instance;
    #endif

    uint32_t cycle;
    uint8_t online;

    void (*daemon_callback)(void*);
    void *device;
} Daemon_Instance;

typedef struct
{
    #if TIM_DAEMON_SUPPORT
    TIM_Init_Config_s tim_config;
    #endif

    uint32_t cycle;

    void (*daemon_callback)(void*);
    void *device;
} Daemon_Init_Config_s;

void Daemon_Reset(Daemon_Instance *daemon);
Daemon_Instance *Daemon_Register(Daemon_Init_Config_s *Daemon_config);
void Daemon_Recall(void *daemon);

#endif