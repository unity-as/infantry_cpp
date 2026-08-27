#include "gimbal_core.h"
#include "bsp_tim.h"
#include <stdlib.h>
#include <string.h>

#define RAD_TO_DEG          57.2957795f
#define GIMBAL_MAX          4

static TIM_Instance    *gc_tim = NULL;
static Gimbal_Instance *gc_list[GIMBAL_MAX] = {0};
static uint8_t          gc_idx = 0;

/*============================================
 *  单轴串级 PID 更新
 *  内置 motor enable/valid 检查，上层无需关心
 ============================================*/

static void Gimbal_AxisUpdate(Gimbal_Axis *axis)
{
    if (!axis->motor->motor_enable || !axis->motor->motor_valid)
        return;

    // 位置模式: 位置环 → vel_target
    if (axis->mode == GIMBAL_POSITION_MODE) {
        if (++(axis->div_cnt) >= axis->pos_freq_div) {
            axis->div_cnt = 0;
            PID_Set_Setpoint(axis->pid_pos, axis->target);
            PID_Update(axis->pid_pos, axis->actual_angle);
            axis->vel_target = axis->pid_pos->output;
        }
    }
    // 速度模式: vel_target 由 API 直接设，跳过位置环

    // 速度环（两种模式共用 vel_target）
    PID_Set_Setpoint(axis->pid_vel, axis->vel_target + axis->vel_ff);
    PID_Set_Feedforward(axis->pid_vel, axis->curr_ff);
    PID_Update(axis->pid_vel, axis->actual_vel);
    DJIMotor_Set_Current(axis->motor, axis->pid_vel->output);
}

/*============================================
 *  定时器回调 — 1ms 周期，遍历所有实例
 ============================================*/

static void Gimbal_Update(void)
{
    for (uint8_t i = 0; i < gc_idx; i++) {
        Gimbal_Instance *gc = gc_list[i];

        // 读取 AHRS 纠偏后角速度 [rad/s] → [deg/s]
        float gyro_yaw   = AHRS_GetGyroZ(gc->ahrs) * RAD_TO_DEG;
        float gyro_pitch = AHRS_GetGyroY(gc->ahrs) * RAD_TO_DEG;

        // 填入 yaw 轴
        gc->yaw.actual_vel   = gyro_yaw;
        gc->yaw.actual_angle  = AHRS_GetYawTotal(gc->ahrs);
        Gimbal_AxisUpdate(&gc->yaw);

        // 填入 pitch 轴
        gc->pitch.actual_vel   = gyro_pitch;
        gc->pitch.actual_angle = AHRS_GetPitch(gc->ahrs);
        Gimbal_AxisUpdate(&gc->pitch);
    }
}

static void Gimbal_TimHandler(void *device)
{
    (void)device;
    Gimbal_Update();
}

/*============================================
 *  Gimbal_Register
 ============================================*/

Gimbal_Instance *Gimbal_Register(Gimbal_Init_Config_s *config)
{
    if (!config || !config->ahrs || gc_idx >= GIMBAL_MAX)
        return NULL;

    Gimbal_Instance *gc = malloc(sizeof(Gimbal_Instance));
    if (!gc) return NULL;
    memset(gc, 0, sizeof(Gimbal_Instance));

    gc->ahrs      = config->ahrs;
    gc->yaw_min   = config->yaw_min;
    gc->yaw_max   = config->yaw_max;
    gc->pitch_min = config->pitch_min;
    gc->pitch_max = config->pitch_max;

    // --- 分频 ---
    gc->yaw.pos_freq_div   = config->pos_freq_div_yaw;
    gc->pitch.pos_freq_div = config->pos_freq_div_pitch;

    // --- 注册电机（内部 PID 清零）---
    DJIMotor_Init_Config_s motor_cfg = {
        .can_handle    = config->can_handle,
        .motor_type    = DJIMotor_6020,
        .motor_id      = config->motor_id_yaw,
        .initial_angle = config->initial_angle_yaw,
        .pid_angle     = { .kp = 0, .ki = 0, .kd = 0 },
        .pid_velocity  = { .kp = 0, .ki = 0, .kd = 0 },
        .pos_freq_div  = 1,
    };
    gc->yaw.motor = DJIMotor_Register(&motor_cfg);

    motor_cfg.motor_id      = config->motor_id_pitch;
    motor_cfg.initial_angle = config->initial_angle_pitch;
    gc->pitch.motor = DJIMotor_Register(&motor_cfg);

    // --- 位置环 PID — 清除 feedforward ---
    gc->yaw.pid_pos   = PID_Init((PID_Init_Config_s *)&config->pid_yaw_pos);
    gc->pitch.pid_pos = PID_Init((PID_Init_Config_s *)&config->pid_pitch_pos);
    PID_Clear_Features(gc->yaw.pid_pos,   PID_FEATURE_FEEDFORWARD);
    PID_Clear_Features(gc->pitch.pid_pos, PID_FEATURE_FEEDFORWARD);

    // --- 速度环 PID — 启用 feedforward ---
    gc->yaw.pid_vel   = PID_Init((PID_Init_Config_s *)&config->pid_yaw_vel);
    gc->pitch.pid_vel = PID_Init((PID_Init_Config_s *)&config->pid_pitch_vel);
    PID_Set_Features(gc->yaw.pid_vel,   PID_FEATURE_FEEDFORWARD);
    PID_Set_Features(gc->pitch.pid_vel, PID_FEATURE_FEEDFORWARD);
    gc->yaw.pid_vel->feedforward_gain   = 1.0f;
    gc->pitch.pid_vel->feedforward_gain = 1.0f;

    // --- 注册到全局列表 ---
    gc_list[gc_idx++] = gc;

    // --- 定时器（首次注册时创建）---
    if (!gc_tim) {
        TIM_Init_Config_s tim_cfg = {
            .htim = config->htim,
            .tim_callback = Gimbal_TimHandler,
            .device = NULL,
        };
        gc_tim = TIM_Register(&tim_cfg);
        TIM_Start_IT(gc_tim);
    }

    return gc;
}

/*============================================
 *  公共 API
 ============================================*/

static float clamp_angle(float angle, float min, float max)
{
    if (min == 0.0f && max == 0.0f) return angle;
    if (angle < min) return min;
    if (angle > max) return max;
    return angle;
}

void Gimbal_Enabale_Pitch(Gimbal_Instance *gc, uint8_t enable)
{
    if (!gc) return;
    DJIMotor_Set_Enable(gc->pitch.motor, enable);
}

void Gimbal_Enabale_Yaw(Gimbal_Instance *gc, uint8_t enable)
{
    if (!gc) return;
    if(gc->yaw.motor)
    DJIMotor_Set_Enable(gc->yaw.motor, enable);
}

void Gimbal_Enable(Gimbal_Instance *gc, uint8_t enable)
{
    if (!gc) return;
    Gimbal_Enabale_Pitch(gc, enable);
    Gimbal_Enabale_Yaw(gc,   enable);
}

void Gimbal_SetTarget(Gimbal_Instance *gc, float yaw, float pitch)
{
    if (!gc) return;
    gc->yaw.mode   = GIMBAL_POSITION_MODE;
    gc->pitch.mode = GIMBAL_POSITION_MODE;
    gc->yaw.target   = yaw;
    gc->yaw.target    = clamp_angle(gc->yaw.target,   gc->yaw_min,   gc->yaw_max);
    gc->pitch.target = pitch;
    gc->pitch.target  = clamp_angle(gc->pitch.target, gc->pitch_min, gc->pitch_max);
}

// 当前角度 + 增量
void Gimbal_SetIncrement(Gimbal_Instance *gc, float yaw_delta, float pitch_delta)
{
    if (!gc) return;
    gc->yaw.mode   = GIMBAL_POSITION_MODE;
    gc->pitch.mode = GIMBAL_POSITION_MODE;
    gc->yaw.target   = yaw_delta + gc->ahrs->output.yaw_total;
    gc->yaw.target    = clamp_angle(gc->yaw.target,   gc->yaw_min,   gc->yaw_max);
    gc->pitch.target = pitch_delta + gc->ahrs->output.euler[1];
    gc->pitch.target  = clamp_angle(gc->pitch.target, gc->pitch_min, gc->pitch_max);
}

void Gimbal_SetPitchAbsolute(Gimbal_Instance *gc, float angle)
{
    if (!gc) return;
    gc->pitch.target = clamp_angle(angle, gc->pitch_min, gc->pitch_max);
}

void Gimbal_SetVelocityFF(Gimbal_Instance *gc, float yaw_ff, float pitch_ff)
{
    if (!gc) return;
    gc->yaw.vel_ff   = yaw_ff;
    gc->pitch.vel_ff = pitch_ff;
}

void Gimbal_SetCurrentFF(Gimbal_Instance *gc, float yaw_ff, float pitch_ff)
{
    if (!gc) return;
    gc->yaw.curr_ff   = yaw_ff;
    gc->pitch.curr_ff = pitch_ff;
}

void Gimbal_SetMode(Gimbal_Instance *gc, Gimbal_Mode yaw_mode, Gimbal_Mode pitch_mode)
{
    if (!gc) return;
    gc->yaw.mode   = yaw_mode;
    gc->pitch.mode = pitch_mode;
}

void Gimbal_SetVelocity(Gimbal_Instance *gc, float yaw_vel, float pitch_vel)
{
    if (!gc) return;

    // // yaw 限幅：超出边界时禁止继续往外走
    // if (gc->yaw.actual_angle > gc->yaw_max && gc->yaw_max != 0.0f) {
    //     if (yaw_vel > 0.0f) yaw_vel = 0.0f;
    // } else if (gc->yaw.actual_angle < gc->yaw_min && gc->yaw_min != 0.0f) {
    //     if (yaw_vel < 0.0f) yaw_vel = 0.0f;
    // }

    // // pitch 限幅
    // if (gc->pitch.actual_angle > gc->pitch_max) {
    //     if (pitch_vel < 0.0f) pitch_vel = 0.0f;
    // } else if (gc->pitch.actual_angle < gc->pitch_min) {
    //     if (pitch_vel > 0.0f) pitch_vel = 0.0f;
    // }

    gc->yaw.mode       = GIMBAL_VELOCITY_MODE;
    gc->pitch.mode     = GIMBAL_VELOCITY_MODE;
    gc->yaw.vel_target   = yaw_vel;
    gc->pitch.vel_target = pitch_vel;
}

float Gimbal_GetYawAngle(Gimbal_Instance *gc)
{
    return gc ? DJIM_GET_ANGLE(gc->yaw.motor) : 0.0f;
}
