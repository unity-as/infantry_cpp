#include "imu_temp.h"
#include <stdlib.h>
#include <string.h>

IMU_Temp_Instance *IMU_Temp_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    IMU_Temp_Instance *inst = (IMU_Temp_Instance *)malloc(sizeof(IMU_Temp_Instance));
    if (!inst) return NULL;
    memset(inst, 0, sizeof(IMU_Temp_Instance));

    inst->target_temp = IMU_TEMP_TARGET;

    PID_Init_Config_s pid_cfg = {
        .kp = 1000.0f,
        .ki = 20.0f,
        .kd = 0.0f,
        .mode = PID_MODE_POSITION,
        .features = PID_FEATURE_INTEGRAL_LIMIT | PID_FEATURE_OUTPUT_LIMIT,
        .integral_limit = 300.0f,
        .output_min = 0.0f,
        .output_max = 2000.0f,
    };
    inst->pid = PID_Init(&pid_cfg);
    if (!inst->pid) { free(inst); return NULL; }
    PID_Set_Setpoint(inst->pid, IMU_TEMP_TARGET);

    PWM_Init_Config_s pwm_cfg = {
        .htim = htim,
        .channel = channel,
    };
    inst->pwm = PWM_Register(&pwm_cfg);
    if (!inst->pwm) { free(inst->pid); free(inst); return NULL; }

    return inst;
}

void IMU_Temp_Update(IMU_Temp_Instance *inst, float temperature)
{
    if (!inst || !inst->pid || !inst->pwm) return;

    if (temperature > IMU_TEMP_SAFETY_MAX || temperature < 0.0f) {
        PWM_Set_DutyRatio(inst->pwm, 0.0f);
        PID_Reset_Integral(inst->pid);
        return;
    }

    PID_Update(inst->pid, temperature);
    PWM_Set_DutyRatio(inst->pwm, inst->pid->output / 4999.0f);
}

uint8_t IMU_Temp_IsReady(IMU_Temp_Instance *inst)
{
    if (!inst || !inst->pid) return 0;
    float err = inst->pid->setpoint - inst->pid->last_feedback;
    return (err > -IMU_TEMP_TOLERANCE && err < IMU_TEMP_TOLERANCE);
}

void IMU_Temp_SafetyOff(IMU_Temp_Instance *inst)
{
    if (inst && inst->pwm)
        PWM_Set_DutyRatio(inst->pwm, 0.0f);
}
