#ifndef IMU_TEMP_H
#define IMU_TEMP_H

#include "bsp_pwm.h"
#include "pid.h"

#define IMU_TEMP_TARGET     40.0f
#define IMU_TEMP_TOLERANCE  1.0f
#define IMU_TEMP_SAFETY_MAX 50.0f

typedef struct {
    PID_Instance *pid;
    PWM_Instance *pwm;
    float target_temp;
} IMU_Temp_Instance;

IMU_Temp_Instance *IMU_Temp_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void IMU_Temp_Update(IMU_Temp_Instance *inst, float temperature);
uint8_t IMU_Temp_IsReady(IMU_Temp_Instance *inst);
void IMU_Temp_SafetyOff(IMU_Temp_Instance *inst);

#endif
