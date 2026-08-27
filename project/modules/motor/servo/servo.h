#ifndef SERVO_H
#define SERVO_H

#include "bsp_pwm.h"

#define SERVO_DEVICE_CNT 2  // 最大支持的舵机实例数量
#define min_duty  0.025f    // 0.5ms / 20ms
#define max_duty  0.125f     // 2.5ms / 20ms

/* 舵机实例结构体 */
typedef struct 
{
    PWM_Instance *servo_pwm;           // PWM实例
} Servo_Instance;

/* 舵机初始化配置 */
typedef struct 
{
    PWM_Init_Config_s servo_pwm_config;   // PWM配置
    float init_angle;               // 初始角度
} Servo_Init_Config_s;

Servo_Instance *ServoRegister(Servo_Init_Config_s *config);
void ServoMove(Servo_Instance *servo, float angle);

#endif // BSP_SERVO_H