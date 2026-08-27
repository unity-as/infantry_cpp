#include "bsp_pwm.h"

#include <string.h>
#include <stdlib.h>

static uint32_t CPU_FREQ_Hz; // CPU频率,在PWM_Set_Period中需要用到

static PWM_Instance *pwm_instance[PWM_DEVICE_CNT] = {0}; // 所有的pwm instance保存于此
static uint8_t idx=0;

PWM_Instance *PWM_Register(PWM_Init_Config_s *PWM_config)
{
    if (idx >= PWM_DEVICE_CNT)
        return NULL; // 超过最大实例数
    if(!idx)
        CPU_FREQ_Hz = HAL_RCC_GetSysClockFreq();
    PWM_Instance *pwm = (PWM_Instance *)malloc(sizeof(PWM_Instance));
    memset(pwm, 0, sizeof(PWM_Instance));

    pwm->htim = PWM_config->htim;
    pwm->channel = PWM_config->channel;
    // 启动PWM
    HAL_TIM_PWM_Start(pwm->htim,pwm->channel);

    pwm_instance[idx++] = pwm;
    return pwm;
}

void PWM_Start(PWM_Instance *_instance)
{
    HAL_TIM_PWM_Start(_instance->htim, _instance->channel);
}

void PWM_Stop(PWM_Instance *_instance)
{
    HAL_TIM_PWM_Stop(_instance->htim, _instance->channel);
}

void PWM_Set_DutyRatio(PWM_Instance *_instance, float dutyratio)
{
    if(dutyratio < 0.0f) dutyratio = 0.0f;
    if(dutyratio > 1.0f) dutyratio = 1.0f;
    __HAL_TIM_SetCompare(_instance->htim,_instance->channel,dutyratio * (_instance->htim->Instance->ARR));
}
void PWM_Set_Period(PWM_Instance *_instance, float period)
{
    uint32_t new_arr = period * CPU_FREQ_Hz /(_instance->htim->Init.Prescaler+1),
    compare = (uint16_t)__HAL_TIM_GetCompare(_instance->htim,_instance->channel) * new_arr/__HAL_TIM_GetAutoreload(_instance->htim);

    __HAL_TIM_SetAutoreload(_instance->htim, new_arr);
    __HAL_TIM_SetCompare(_instance->htim,_instance->channel,compare);
}