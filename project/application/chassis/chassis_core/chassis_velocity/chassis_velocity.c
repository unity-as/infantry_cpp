#include "chassis_velocity.h"

#include <stdlib.h>
#include <string.h>

// 反算抗饱和：功率控制限幅后回灌积分，integral -= (1-scale)*I_raw*coef
static void ChassisVelocity_BackCalc(ChassisVelocity_Instance *instance, float scale)
{
    if (scale >= 1.0f) return;   // 未限流，无需反算

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        float back = (1.0f - scale) * instance->current_raw[i] * instance->back_calc_coef;
        PID_Set_Integral(instance->pid_velocity[i],
                         instance->pid_velocity[i]->integral - back);
    }
}

ChassisVelocity_Instance *ChassisVelocity_Register(ChassisVelocity_Init_Config_s *config)
{
    if (!config) return NULL;

    ChassisVelocity_Instance *instance = (ChassisVelocity_Instance *)malloc(sizeof(ChassisVelocity_Instance));
    if (!instance) return NULL;
    memset(instance, 0, sizeof(ChassisVelocity_Instance));

    instance->back_calc_coef = config->back_calc_coef;
    instance->power_limit = 0.0f;   // 默认不限流，应用层覆盖
    instance->scale = 1.0f;

    // 注册四轮电机 + 速度环，轮序 [rf, lf, lr, rr]
    DJIMotor_Init_Config_s m_cfg = {
        .can_handle = config->can_handle,
        .motor_type = DJIMotor_3508,
        .reduction_ratio = config->reduction_ratio > 0.0f ? config->reduction_ratio : 1.0f,
    };
    uint8_t motor_id[CHASSIS_WHEEL_NUM] = {
        config->motor_id_rf, config->motor_id_lf, config->motor_id_lr, config->motor_id_rr
    };
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        m_cfg.motor_id = motor_id[i];
        instance->motor[i] = DJIMotor_Register(&m_cfg);
        instance->pid_velocity[i] = PID_Init(&config->pid_velocity);
    }

    return instance;
}

void ChassisVelocity_SetTarget(ChassisVelocity_Instance *instance, const float omega_deg[CHASSIS_WHEEL_NUM])
{
    if (!instance) return;
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        instance->target_omega[i] = omega_deg[i];
}

void ChassisVelocity_Tick(ChassisVelocity_Instance *instance)
{
    if (!instance) return;

    float scale = instance->scale;   // 失能时保持上一次缩放比（current_raw 已在 SetEnable 清 0）

    // 速度环 → 预测功率 → 缩放 → 限幅发电流 → 反算：仅使能时跑
    if (instance->enable) {
        // 1. 速度环：目标速度 → I_raw
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
            PID_Set_Setpoint(instance->pid_velocity[i], instance->target_omega[i]);
            PID_Update(instance->pid_velocity[i], DJIM_GET_VELOCITY(instance->motor[i]));
            instance->current_raw[i] = instance->pid_velocity[i]->output;
        }

        // 2. 预测功率：P = Σ Kt·I_raw·ω（负功率/倒灌算 0）
        float power_pred = 0.0f;
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
            power_pred += fmaxf(0.0f, CHASSIS_POWER_COEF * instance->current_raw[i] * DJIM_GET_VELOCITY(instance->motor[i]));

        // 3. 缩放比：limit<=0 放开
        scale = 1.0f;
        if (instance->power_limit > 0.0f && power_pred > instance->power_limit)
            scale = instance->power_limit / power_pred;
        if (scale > 1.0f) scale = 1.0f;
        if (scale < 0.0f) scale = 0.0f;
        instance->scale = scale;

        // 4. 限幅电流 + 发电流
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
            DJIMotor_Set_Current(instance->motor[i], scale * instance->current_raw[i]);

        // 5. 反算抗饱和
        ChassisVelocity_BackCalc(instance, scale);
    }

    // 6. 实测总功率（监测）—— 不受使能影响，持续跑
    instance->power_total = CHASSIS_GET_POWER(instance->motor[0])
                          + CHASSIS_GET_POWER(instance->motor[1])
                          + CHASSIS_GET_POWER(instance->motor[2])
                          + CHASSIS_GET_POWER(instance->motor[3]);

    //DEBUG
    static float max_power = 0.0f;
    static float power_out = 0.0f;
    static float max_outpower = 0.0f;
    power_out = fmaxf(0.0f, CHASSIS_POWER_COEF * scale * instance->current_raw[0] * DJIM_GET_VELOCITY(instance->motor[0]))
                + fmaxf(0.0f, CHASSIS_POWER_COEF * scale * instance->current_raw[1] * DJIM_GET_VELOCITY(instance->motor[1]))
                + fmaxf(0.0f, CHASSIS_POWER_COEF * scale * instance->current_raw[2] * DJIM_GET_VELOCITY(instance->motor[2]))
                + fmaxf(0.0f, CHASSIS_POWER_COEF * scale * instance->current_raw[3] * DJIM_GET_VELOCITY(instance->motor[3]));
    if(power_out > max_outpower)
        max_outpower = power_out;
    if(instance->power_total > max_power)
        max_power = instance->power_total;
}

void ChassisVelocity_SetEnable(ChassisVelocity_Instance *instance, uint8_t enable)
{
    if (!instance) return;
    instance->enable = enable ? 1 : 0;
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        DJIMotor_Set_Enable(instance->motor[i], instance->enable);

    // 失能时清一次：复位速度环 PID + 清中间量，防停转期间积分饱和、使能瞬间冲电流
    if (!instance->enable) {
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
            PID_Reset(instance->pid_velocity[i]);
            instance->current_raw[i] = 0.0f;
        }
        instance->scale = 1.0f;
    }
}

void ChassisVelocity_SetPowerLimit(ChassisVelocity_Instance *instance, float limit)
{
    if (!instance) return;
    instance->power_limit = limit;
}

float ChassisVelocity_GetPower(ChassisVelocity_Instance *instance)
{
    if (!instance) return 0.0f;
    return instance->power_total;
}
