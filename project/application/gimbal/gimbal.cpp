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
#include "SEGGER_RTT.h"

#define GIMBAL_PITCH_CURRENT_FF  -0.150f

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
        .can_handle          = &hcan2,
        .htim                = &htim5,
        .motor_id_yaw        = 1,
        .motor_id_pitch      = 2,
        .initial_angle_yaw   = GIMBAL_ECD_TO_DEG(GIMBAL_YAW_ECD),
        .initial_angle_pitch = GIMBAL_ECD_TO_DEG(GIMBAL_PITCH_ECD),

        .pid_yaw_pos = {
            .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID::Mode::Position,
            .features = PID::FeatureOutputLimit,
            .output_min = -480.0f, .output_max = 480.0f,
        },
        .pid_pitch_pos = {
            .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID::Mode::Position,
            .features = PID::FeatureOutputLimit
                    //   | PID::FeatureFilter
                      | PID::FeatureIntegralLimit,
            .integral_limit = 0.0f,
            .output_min = -240.0f, .output_max = 240.0f,
            .filter_alpha = 0.3f,
        },
        .pid_yaw_vel = {
            .kp = 0.04f, .ki = 0.0005f, .kd = 0.02f,
            .mode = PID::Mode::Position,
            .features = PID::FeatureOutputLimit
                      | PID::FeatureIntegralLimit
                      | PID::FeatureDerivativeOnMeasurement
                    //   | PID::FeatureVariableGain
                      | PID::FeatureFilter
                      ,
            .integral_limit = 200.0f,
            .output_min = -1.0f, .output_max = 1.0f,
            .filter_alpha = 0.2f,
            .kp_extra = 0.00001f,
        },
        .pid_pitch_vel = {
            .kp = 0.007f, .ki = 0.00006f, .kd = 0.002f,
            .mode = PID::Mode::Position,
            .features = PID::FeatureOutputLimit
                      | PID::FeatureIntegralLimit
                      | PID::FeatureDerivativeOnMeasurement
                      | PID::FeatureFilter
                    ,
            .integral_limit = 5000.0f,
            .output_min = -1.0f, .output_max = 1.0f,
            .filter_alpha = 0.5f,
        },

        .pos_freq_div_yaw   = 5,
        .pos_freq_div_pitch = 5,

        .yaw_min = 0.0f, .yaw_max = 0.0f,
        .pitch_min = GIMBAL_PITCH_MIN,
        .pitch_max = GIMBAL_PITCH_MAX,
    };

    gc.init(cfg);

    // --- 自瞄 PID：参数沿用云台位置 PID ---
    PID::Config aim_yaw_cfg =
    {
        .kp = 6.0f, .ki = 0.0f, .kd = 0.01f,
        .mode = PID::Mode::Position,
        .features = PID::FeatureOutputLimit
                    // | PID::FeatureFilter
                    | PID::FeatureIntegralLimit,
        .integral_limit = 180.0f,
        .output_min = -360.0f, .output_max = 360.0f,
        .filter_alpha = 0.5f,
    };
    aim_pid_yaw.init(aim_yaw_cfg);

    PID::Config aim_pitch_cfg =
    {
        .kp = 8.0, .ki = 0.0f, .kd = 0.01f,
        .mode = PID::Mode::Position,
        .features = PID::FeatureOutputLimit
                    // | PID::FeatureFilter
                    | PID::FeatureIntegralLimit,
        .integral_limit = 180.0f,
        .output_min = -360.0f, .output_max = 360.0f,
        .filter_alpha = 0.5f,
    };
    aim_pid_pitch.init(aim_pitch_cfg);

    gc.setCurrentFF(0.0f, GIMBAL_PITCH_CURRENT_FF);  // 重力前馈(标定值)

    const osThreadAttr_t attr = {
        .name = "Gimbal",
        .stack_size = 1024,
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
    uint16_t dbg_div = 0;
    for (;;) {
        // 重新使能: 先同步目标再使能，避免定时器拿旧目标跑
        if (gimbal_cmd.enable && !last_en) {
            Gimbal_SyncTarget();
        }
        gc.enable(gimbal_cmd.enable);
        last_en = gimbal_cmd.enable;

        // 调参打印: ~10ms 一次, pitch 速度环 (vt/av ×10, out ×1000, I ×100)
        if (++dbg_div >= 10) {
            dbg_div = 0;
            SEGGER_RTT_printf(0, "P vt=%d av=%d out=%d I=%d ang=%d\r\n",
                (int)(gc.pitch_.vel_target * 10.0f),
                (int)(gc.pitch_.actual_vel * 10.0f),
                (int)(gc.pitch_.pid_vel.output_ * 1000.0f),
                (int)(gc.pitch_.pid_vel.integral_ * 100.0f),
                (int)(gc.pitch_.actual_angle * 10.0f));
        }
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
}

void Gimbal_SyncTarget(void)
{
    gc.setTarget(gc.ahrs_->output_.yaw_total, gc.ahrs_->output_.euler[1]);
    // 清速度目标，防止速度环用自瞄旧值跑
    gc.yaw_.vel_target   = 0.0f;
    gc.pitch_.vel_target = 0.0f;
}

void Gimbal_AutoAim(float yaw_err, float pitch_err)
{
    static uint8_t div = 0;
    if (++div < 10) return;
    div = 0;

    // 限制误差范围，防止云台打到极限位置
    if (gc.pitch_.actual_angle - gc.pitch_max_ > pitch_err)
        pitch_err = gc.pitch_.actual_angle - gc.pitch_max_;
    else if (gc.pitch_.actual_angle - gc.pitch_min_ < pitch_err)
        pitch_err = gc.pitch_.actual_angle - gc.pitch_min_;

    // 自瞄 PID: 误差 → 速度指令
    aim_pid_yaw.update(yaw_err);
    aim_pid_pitch.update(pitch_err);

    // 速度指令 → gimbal_core 速度环（定时器驱动）
    gc.setVelocity(aim_pid_yaw.output_, aim_pid_pitch.output_);
}
