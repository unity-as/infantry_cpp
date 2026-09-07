/**
 * @file    motor_test.cpp
 * @brief   测试DJIMotor时用的应用层，已完成测试，作废
 * @note    从 C 版 motor_test 迁移，逻辑不变，禁堆。
 *          原 debug 打印里的 pid_angle->output 为 DJIMotor 私有成员，已省略该字段。
 */
#include "motor_test.h"
#include "dji_motor.h"
#include "SEGGER_RTT.h"

static DJIMotor motor_yaw;
static uint32_t last_step_tick = 0;
static uint32_t last_log_tick = 0;
static float target = 0.0f;

void MotorTest_Init(void)
{
    SEGGER_RTT_printf(0, "\r\n=== YAW 45deg STEP (1s) ===\r\n");

    DJIMotor::Config cfg = {
        .can_handle = &hcan2,
        .motor_id = 1,
        .motor_type = DJIMotor_6020,
        .pid_angle = {
            .kp = 12.0f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID::Mode::Position,
            .features = PID::FeatureOutputLimit,
            .output_min = -360.0f, .output_max = 360.0f,
        },
        .pid_velocity = {
            .kp = 0.01f, .ki = 0.02f, .kd = 0.0f,
            .mode = PID::Mode::Position,
            .features = PID::FeatureOutputLimit
                      | PID::FeatureIntegralLimit
                      | PID::FeatureFilter,
            .integral_limit = 1.0f,
            .output_min = -1.0f, .output_max = 1.0f,
            .filter_alpha = 0.3f,
        },
        .initial_angle = 232.91f,
    };

    motor_yaw.init(cfg);
    motor_yaw.setEnable(1);
    motor_yaw.setAngle(target);

    SEGGER_RTT_printf(0, "  Yaw -> %s\r\n", "OK");
    SEGGER_RTT_printf(0, "=== READY ===\r\n\r\n");
}

void MotorTest_Update(void)
{
    uint32_t tick = HAL_GetTick();

    // 每秒跳45度
    if (tick - last_step_tick > 1000) {
        last_step_tick = tick;
        target += 45.0f;
        motor_yaw.setAngle(target);
    }

    if (tick - last_log_tick < 200) return;
    last_log_tick = tick;

    SEGGER_RTT_printf(0, "Yaw: tgt=%d fb=%d err=%d\r\n",
        (int)target,
        (int)motor_yaw.angle_,
        (int)(motor_yaw.target_angle_ - motor_yaw.angle_));
}
