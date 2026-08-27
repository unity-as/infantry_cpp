#include "gimbal.h"
#include "gimbal_core.h"
#include "cmd.h"
#include "config.h"
#include "pid.h"
#include "cmsis_os2.h"
#include "SEGGER_RTT.h"

#define GIMBAL_PITCH_CURRENT_FF  -0.150f

Gimbal_Cmd gimbal_cmd;
static Gimbal_Instance *gc;
static PID_Instance *aim_pid_yaw;
static PID_Instance *aim_pid_pitch;

static void Gimbal_Task(void *arg);

/*============================================
 *  Gimbal_Init
 ============================================*/

void Gimbal_Init(void)
{
    Gimbal_Init_Config_s cfg = {
        .ahrs                = cmd_ahrs,
        .can_handle          = &hcan2,
        .htim                = &htim5,
        .motor_id_yaw        = 1,
        .motor_id_pitch      = 2,
        .initial_angle_yaw   = GIMBAL_ECD_TO_DEG(GIMBAL_YAW_ECD),
        .initial_angle_pitch = GIMBAL_ECD_TO_DEG(GIMBAL_PITCH_ECD),

        .pid_yaw_pos = {
            .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID_MODE_POSITION,
            .features = PID_FEATURE_OUTPUT_LIMIT,
            .output_min = -480.0f, .output_max = 480.0f,
        },
        .pid_yaw_vel = {
            .kp = 0.04f, .ki = 0.0005f, .kd = 0.02f,
            .mode = PID_MODE_POSITION,
            .features = PID_FEATURE_OUTPUT_LIMIT
                      | PID_FEATURE_INTEGRAL_LIMIT
                      | PID_FEATURE_DERIVATIVE_ON_MEASUREMENT
                    //   | PID_FEATURE_VARIABLE_GAIN
                      | PID_FEATURE_FILTER
                      ,
            .kp_extra = 0.00001f,
            .output_min = -1.0f, .output_max = 1.0f,
            .integral_limit = 200.0f, .filter_alpha = 0.2f,
        },
        .pid_pitch_pos = {
            .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID_MODE_POSITION,
            .features = PID_FEATURE_OUTPUT_LIMIT
                    //   | PID_FEATURE_FILTER
                      | PID_FEATURE_INTEGRAL_LIMIT,
            .output_min = -240.0f, .output_max = 240.0f,
            .integral_limit = 0.0f, .filter_alpha = 0.3f,
        },
        .pid_pitch_vel = {
            .kp = 0.007f, .ki = 0.00006f, .kd = 0.002f,
            .mode = PID_MODE_POSITION,
            .features = PID_FEATURE_OUTPUT_LIMIT
                      | PID_FEATURE_INTEGRAL_LIMIT
                      | PID_FEATURE_DERIVATIVE_ON_MEASUREMENT
                      | PID_FEATURE_FILTER
                    ,
            .output_min = -1.0f, .output_max = 1.0f,
            .integral_limit = 5000.0f, .filter_alpha = 0.5f,
        },

        .pos_freq_div_yaw   = 5,
        .pos_freq_div_pitch = 5,

        .yaw_min = 0.0f, .yaw_max = 0.0f,
        .pitch_min = GIMBAL_PITCH_MIN,
        .pitch_max = GIMBAL_PITCH_MAX,
    };

    gc = Gimbal_Register(&cfg);

    // --- 自瞄 PID：参数沿用云台位置 PID ---
    PID_Init_Config_s aim_yaw_cfg =
    {
        .kp = 6.0f, .ki = 0.0f, .kd = 0.01f,
        .mode = PID_MODE_POSITION,
        .features = PID_FEATURE_OUTPUT_LIMIT
                    // | PID_FEATURE_FILTER
                    | PID_FEATURE_INTEGRAL_LIMIT,
        .output_min = -360.0f, .output_max = 360.0f,
        .integral_limit = 180.0f, .filter_alpha = 0.5f,
    };
    aim_pid_yaw = PID_Init(&aim_yaw_cfg);
    PID_Init_Config_s aim_pitch_cfg =
    {
        .kp = 8.0, .ki = 0.0f, .kd = 0.01f,
        .mode = PID_MODE_POSITION,
        .features = PID_FEATURE_OUTPUT_LIMIT
                    // | PID_FEATURE_FILTER
                    | PID_FEATURE_INTEGRAL_LIMIT,
        .output_min = -360.0f, .output_max = 360.0f,
        .integral_limit = 180.0f, .filter_alpha = 0.5f,
    };
    aim_pid_pitch = PID_Init(&aim_pitch_cfg);

    Gimbal_SetCurrentFF(gc, 0.0f, GIMBAL_PITCH_CURRENT_FF);  // 重力前馈(标定值)
    const osThreadAttr_t attr = {
        .name = "Gimbal",
        .stack_size = 1024,
        .priority = osPriorityNormal,
    };
    osThreadNew(Gimbal_Task, NULL, &attr);
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
        Gimbal_Enable(gc, gimbal_cmd.enable);
        last_en = gimbal_cmd.enable;

        // 调参打印: ~10ms 一次, pitch 速度环 (vt/av ×10, out ×1000, I ×100)
        if (++dbg_div >= 10) {
            dbg_div = 0;
            SEGGER_RTT_printf(0, "P vt=%d av=%d out=%d I=%d ang=%d\r\n",
                (int)(gc->pitch.vel_target * 10.0f),
                (int)(gc->pitch.actual_vel * 10.0f),
                (int)(gc->pitch.pid_vel->output * 1000.0f),
                (int)(gc->pitch.pid_vel->integral * 100.0f),
                (int)(gc->pitch.actual_angle * 10.0f));
        }
        osDelay(1);
    }
}

/*============================================
 *  cmd.c 兼容翻译 API 封装层
 ============================================*/

void Gimbal_Set_Increment(float yaw_delta, float pitch_delta)
{
    Gimbal_SetIncrement(gc, yaw_delta, pitch_delta);
}

void Gimbal_Set_Increment_Setpoint(float yaw, float pitch)
{
    Gimbal_SetTarget(gc, gc->yaw.target + yaw, gc->pitch.target + pitch);
}

void Gimbal_Set_Mode(uint8_t yaw_mode, uint8_t pitch_mode)
{
    Gimbal_SetMode(gc, (Gimbal_Mode)yaw_mode, (Gimbal_Mode)pitch_mode);
}

void Gimbal_Set_Velocity(float yaw_vel, float pitch_vel)
{
    Gimbal_SetVelocity(gc, yaw_vel, pitch_vel);
}

float Gimbal_GetYaw(void)
{
    return Gimbal_GetYawAngle(gc);
}

void Gimbal_AutoAim_Reset(void)
{
    PID_Reset(aim_pid_yaw);
    PID_Reset(aim_pid_pitch);
}

void Gimbal_SyncTarget(void)
{
    Gimbal_SetTarget(gc,
        gc->ahrs->output.yaw_total,
        gc->ahrs->output.euler[1]);
    // 清速度目标，防止速度环用自瞄旧值跑
    gc->yaw.vel_target   = 0.0f;
    gc->pitch.vel_target = 0.0f;
}

void Gimbal_AutoAim(float yaw_err, float pitch_err)
{
    static uint8_t div = 0;
    if (++div < 10) return;
    div = 0;

    // 限制误差范围，防止云台打到极限位置
    if (gc->pitch.actual_angle - gc->pitch_max > pitch_err)
        pitch_err = gc->pitch.actual_angle - gc->pitch_max;
    else if (gc->pitch.actual_angle - gc->pitch_min < pitch_err)
        pitch_err = gc->pitch.actual_angle - gc->pitch_min;

    // 自瞄 PID: 误差 → 速度指令
    PID_Update(aim_pid_yaw, yaw_err);
    PID_Update(aim_pid_pitch, pitch_err);

    // 速度指令 → gimbal_core 速度环（定时器驱动）
    Gimbal_SetVelocity(gc, aim_pid_yaw->output, aim_pid_pitch->output);
}
