/**
 * @file motor_test.c
 * @brief 测试DJIMotor时用的应用层，已完成测试，作废
 */

#include "motor_test.h"
#include "dji_motor.h"
#include "SEGGER_RTT.h"

static DJIMotor_Instance *motor_yaw = NULL;
static uint32_t last_step_tick = 0;
static uint32_t last_log_tick = 0;
static float target = 0.0f;

void MotorTest_Init(void)
{
    SEGGER_RTT_printf(0, "\r\n=== YAW 45deg STEP (1s) ===\r\n");

    DJIMotor_Init_Config_s cfg = {
        .can_handle = &hcan2,
        .motor_type = DJIMotor_6020,
        .motor_id = 1,
        .initial_angle = 232.91f,
        .pid_angle = {
            .kp = 12.0f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID_MODE_POSITION,
            .features = PID_FEATURE_OUTPUT_LIMIT,
            .output_min = -360.0f, .output_max = 360.0f,
        },
        .pid_velocity = {
            .kp = 0.01f, .ki = 0.02f, .kd = 0.0f,
            .mode = PID_MODE_POSITION,
            .features = PID_FEATURE_OUTPUT_LIMIT
                      | PID_FEATURE_INTEGRAL_LIMIT
                      | PID_FEATURE_FILTER,
            .output_min = -1.0f, .output_max = 1.0f,
            .integral_limit = 1.0f, .filter_alpha = 0.3f,
        },
    };

    motor_yaw = DJIMotor_Register(&cfg);
    if (motor_yaw) {
        DJIMotor_Set_Enable(motor_yaw, 1);
        DJIMotor_Set_Angle(motor_yaw, target);
    }
    SEGGER_RTT_printf(0, "  Yaw -> %s\r\n", motor_yaw ? "OK" : "FAIL");
    SEGGER_RTT_printf(0, "=== READY ===\r\n\r\n");
}

void MotorTest_Update(void)
{
    uint32_t tick = HAL_GetTick();

    // 每秒跳45度
    if (tick - last_step_tick > 1000) {
        last_step_tick = tick;
        target += 45.0f;
        if (motor_yaw) DJIMotor_Set_Angle(motor_yaw, target);
    }

    if (tick - last_log_tick < 200) return;
    last_log_tick = tick;

    if (motor_yaw) {
        SEGGER_RTT_printf(0, "Yaw: tgt=%d fb=%d err=%d out=%d\r\n",
            (int)target,
            (int)DJIM_GET_ANGLE(motor_yaw),
            (int)(motor_yaw->target_angle - motor_yaw->angle),
            (int)motor_yaw->pid_angle->output);
    }
}
