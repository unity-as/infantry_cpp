/**
 * @file    gimbal.cpp
 * @brief   云台应用层（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 gimbal 迁移，逻辑不变，禁堆（gc/aim_pid 由指针改为全局对象）。
 */
#include "gimbal.h"
#include "gimbal_core.h"
#include "cmd.h"
#include "config.h"
#include "pid.h"
#include "cmsis_os2.h"

Gimbal_Cmd gimbal_cmd;
static Gimbal gc;
static PID aim_pid_yaw;
static PID aim_pid_pitch;

static void Gimbal_Task(void *arg);

/*============================================
 *  Gimbal_Init
 ============================================*/

void Gimbal_Init(void)
{
    Gimbal::Config cfg = {
        .ahrs                = &cmd_ahrs,
        .can_handle          = GIMBAL_CAN_HANDLE,
        .htim                = ROBOT_MOTOR_TIM_HANDLE,
        .motor_id_yaw        = GIMBAL_MOTOR_ID_YAW,
        .motor_id_pitch      = GIMBAL_MOTOR_ID_PITCH,
        .initial_angle_yaw   = GIMBAL_ECD_TO_DEG(GIMBAL_YAW_ECD),
        .initial_angle_pitch = GIMBAL_ECD_TO_DEG(GIMBAL_PITCH_ECD),

        .pid_yaw_pos = GIMBAL_YAW_POSITION_PID,
        .pid_pitch_pos = GIMBAL_PITCH_POSITION_PID,
        .pid_yaw_vel = GIMBAL_YAW_VELOCITY_PID,
        .pid_pitch_vel = GIMBAL_PITCH_VELOCITY_PID,

        .pos_freq_div_yaw   = GIMBAL_POS_FREQ_DIV_YAW,
        .pos_freq_div_pitch = GIMBAL_POS_FREQ_DIV_PITCH,

        .yaw_min = GIMBAL_YAW_AHRS_MIN, .yaw_max = GIMBAL_YAW_AHRS_MAX,
        .pitch_min = GIMBAL_PITCH_AHRS_MIN,
        .pitch_max = GIMBAL_PITCH_AHRS_MAX,
        .pitch_motor_min = GIMBAL_PITCH_MOTOR_MIN,
        .pitch_motor_max = GIMBAL_PITCH_MOTOR_MAX,
    };

    gc.init(cfg);

    // 自瞄误差 → 速度的 PID 与云台位置环分别配置。
    aim_pid_yaw.init(GIMBAL_AIM_YAW_PID);
    aim_pid_pitch.init(GIMBAL_AIM_PITCH_PID);

    gc.setCurrentFF(GIMBAL_YAW_CURRENT_FF, GIMBAL_PITCH_CURRENT_FF);  // 重力前馈(标定值)

    const osThreadAttr_t attr = {
        .name = "Gimbal",
        .stack_size = GIMBAL_TASK_STACK_BYTES,
        .priority = osPriorityNormal,
    };
    osThreadNew(Gimbal_Task, nullptr, &attr);
}

/*============================================
 *  应用层任务 — 收 CMD → 调 core API
 ============================================*/

static void Gimbal_Task(void *arg)
{
    (void)arg;
    uint8_t last_en = 0;
    for (;;) {
        // 重新使能: 先同步目标再使能，避免定时器拿旧目标跑
        if (gimbal_cmd.enable && !last_en) {
            Gimbal_SyncTarget();
        }
        gc.enable(gimbal_cmd.enable);
        last_en = gimbal_cmd.enable;
        osDelay(1);
    }
}

/*============================================
 *  cmd.c 兼容翻译 API 封装层
 ============================================*/

void Gimbal_Set_Increment(float yaw_delta, float pitch_delta)
{
    gc.setIncrement(yaw_delta, pitch_delta);
}

void Gimbal_Set_Increment_Setpoint(float yaw, float pitch)
{
    gc.setTarget(gc.yaw_.target + yaw, gc.pitch_.target + pitch);
}

void Gimbal_Set_Mode(uint8_t yaw_mode, uint8_t pitch_mode)
{
    gc.setMode(static_cast<Gimbal::Mode>(yaw_mode), static_cast<Gimbal::Mode>(pitch_mode));
}

void Gimbal_Set_Velocity(float yaw_vel, float pitch_vel)
{
    gc.setVelocity(yaw_vel, pitch_vel);
}

float Gimbal_GetYaw(void)
{
    return gc.getYawAngle();
}

void Gimbal_AutoAim_Reset(void)
{
    aim_pid_yaw.reset();
    aim_pid_pitch.reset();
    gc.setVelocityFF(0.0f, 0.0f);
}

void Gimbal_SyncTarget(void)
{
    gc.setVelocityFF(0.0f, 0.0f);
    gc.setTarget(gc.ahrs_->output_.yaw_total, gc.ahrs_->output_.euler[1]);
    // 清速度目标，防止速度环用自瞄旧值跑
    gc.yaw_.vel_target   = 0.0f;
    gc.pitch_.vel_target = 0.0f;
}

void Gimbal_AutoAim(float yaw_err, float pitch_err, float yaw_ff, float pitch_ff)
{
    static uint8_t div = 0;
    if (++div < GIMBAL_AIM_FREQ_DIV) return;
    div = 0;

    // 自瞄输入为“当前 - 目标”，转换为“目标 - 当前”后复用双重限位。
    pitch_err = -gc.limitPitchError(-pitch_err);

    // 自瞄 PID: 误差 → 速度指令
    aim_pid_yaw.update(yaw_err);
    aim_pid_pitch.update(pitch_err);

    // 速度指令 → gimbal_core 速度环（定时器驱动）
    gc.setVelocity(aim_pid_yaw.output_, aim_pid_pitch.output_);
    gc.setVelocityFF(GIMBAL_AIM_FF_GAIN * yaw_ff, GIMBAL_AIM_FF_GAIN * pitch_ff);
}
