#include "bsp_tim.h"
#include <stdlib.h>
#include <string.h>

static TIM_Instance *tim_instance[TIM_MX_DEVICE_NUM];
static uint8_t idx;

TIM_Instance *TIM_Register(TIM_Init_Config_s *TIM_config)
{
    if (idx >= TIM_MX_DEVICE_NUM)
        return NULL;

    TIM_Instance *instance=(TIM_Instance*)malloc(sizeof(TIM_Instance));
    memset(instance,0,sizeof(TIM_Instance));
    instance->htim=TIM_config->htim;
    instance->tim_callback = TIM_config->tim_callback;
    instance->device = TIM_config->device;
    tim_instance[idx++]=instance;

    TIM_Start_IT(instance);

    return instance;
}

void TIM_Start_IT(TIM_Instance *tim)
{
    HAL_TIM_Base_Start_IT(tim->htim);
}

void TIM_Stop_IT(TIM_Instance *tim)
{
    HAL_TIM_Base_Stop_IT(tim->htim);
}

void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//选用定时器时基时，原本的弱定义函数会被main.c定义，要在那边调用这个函数
{
    TIM_Instance *tim;
    for(uint8_t i=0;i<idx;i++)
    {
        tim=tim_instance[i];
        if(tim->tim_callback !=NULL && tim->htim==htim)
            tim->tim_callback(tim->device);
    }
}