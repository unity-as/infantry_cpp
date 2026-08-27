#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "tim.h"
__TIM_H__

#define TIM_MX_DEVICE_NUM 25

typedef struct tim
{
    TIM_HandleTypeDef *htim;
    void (*tim_callback)(void*);
    void *device;
} TIM_Instance;

typedef struct
{
    TIM_HandleTypeDef *htim;
    void (*tim_callback)(void*);
    void *device;
} TIM_Init_Config_s;

void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

TIM_Instance *TIM_Register(TIM_Init_Config_s *TIM_config);
void TIM_Start_IT(TIM_Instance *tim);
void TIM_Stop_IT(TIM_Instance *tim);

#endif