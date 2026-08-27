#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"

#define PWM_DEVICE_CNT 10 // 最大支持的PWM实例数量

//pwm实例
typedef struct
{
    TIM_HandleTypeDef *htim;              // TIM句柄
    uint32_t channel;                     // 通道
} PWM_Instance;

//pwm初始模板
typedef struct
{
    TIM_HandleTypeDef *htim;              // TIM句柄
    uint32_t channel;                     // 通道
} PWM_Init_Config_s;


PWM_Instance *PWM_Register(PWM_Init_Config_s *PWM_config);
void PWM_Start(PWM_Instance *pwm);
void PWM_Set_DutyRatio(PWM_Instance *pwm, float dutyratio);
void PWM_Stop(PWM_Instance *pwm);
void PWM_Set_Period(PWM_Instance *pwm, float period);

 #endif // BSP_PWM_H