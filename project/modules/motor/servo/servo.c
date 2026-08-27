#include "servo.h"

#include <string.h>
#include <stdlib.h>

static Servo_Instance *servo_instance[SERVO_DEVICE_CNT] = {NULL};
static uint8_t idx = 0;

static float AngleToDuty(Servo_Instance *servo, float angle)
{    
    // 限制角度在范围内
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;
    
    // 角度 -> 占空比
    float ratio = (angle - 0) / (180 - 0);
    return 5.0 + ratio * (10.0 - 5.0); // 0.5ms ~ 2.5ms 对应 5% ~ 10% 占空比
}

Servo_Instance *ServoRegister(Servo_Init_Config_s *config)
{
    Servo_Instance *servo = (Servo_Instance *)malloc(sizeof(Servo_Instance));
    memset(servo, 0, sizeof(Servo_Instance));

    // 注册PWM实例
    servo->servo_pwm = PWM_Register(&config->servo_pwm_config);

    float init_duty = AngleToDuty(servo, config->init_angle);
    PWM_Set_DutyRatio(servo->servo_pwm, init_duty);

    servo_instance[idx++] = servo;
    return servo;
}

void ServoMove(Servo_Instance *servo, float angle)
{
    PWM_Set_DutyRatio(servo->servo_pwm, AngleToDuty(servo, angle));
}