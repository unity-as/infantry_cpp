/**
 * @file    chassis.cpp
 * @brief   底盘应用层：读 chassis_cmd，灌入 ChassisMotion（core）
 */
#include "chassis.h"
#include "config.h"
#include "chassis_motion.h"
#include "cmsis_os2.h"
#include "referee.h"
#include <math.h>

Chassis_Cmd chassis_cmd;
static ChassisMotion chassis_inst;

static void Chassis_Task(void *arg)
{
    (void)arg;
    for (;;) {
        const referee_info_t* referee_data = Referee_GetData();
        float limit = CHASSIS_POWER_FALLBACK_W;
        const float referee_limit = referee_data->robot_status.chassis_power_limit;
        if (Referee_Online() && referee_limit > 0.0f) {
            limit = referee_limit - CHASSIS_POWER_MARGIN_W
                  + CHASSIS_BUFFER_GAIN_W_PER_J * (referee_data->power_heat.buffer_energy - CHASSIS_BUFFER_TARGET_J);
            limit = fminf(referee_limit, fmaxf(0.0f, limit));
        }
        chassis_inst.velocity_.setPowerLimit(limit);

        chassis_inst.velocity_.setEnable(chassis_cmd.enable);
        chassis_inst.v_ = chassis_cmd.v;
        chassis_inst.theta_ = chassis_cmd.theta;

        if (chassis_cmd.mode == CHASSIS_MODE_FOLLOW)
        {
            // float error = fmodf(chassis_cmd.yaw_motor_angle + 180.0f, 360.0f);
            // if (error < 0.0f) error += 360.0f;   // fmodf 结果符号与被除数一致，负数时需归一到 [0,360)
            // error -= 180.0f;
            float error = fmodf(chassis_cmd.yaw_motor_angle + CHASSIS_FOLLOW_PERIOD_DEG / 2.0f, CHASSIS_FOLLOW_PERIOD_DEG);
            if (error < 0.0f) error += CHASSIS_FOLLOW_PERIOD_DEG;   // fmodf 负结果归一到一个跟随周期内
            error -= CHASSIS_FOLLOW_PERIOD_DEG / 2.0f;
            // chassis_inst.w_rot_ = chassis_inst.theta_ * 2.0f;//跟随底盘移动方向
            chassis_inst.w_rot_ = error * CHASSIS_FOLLOW_GAIN;//跟随云台方向

            //纯P就懒得上pid了
        }
        else if (chassis_cmd.mode == CHASSIS_MODE_NO_ROTATION)
            chassis_inst.w_rot_ = chassis_cmd.w_rot;//手动角速度（拨轮）
        else
            chassis_inst.w_rot_ = 0.0f;//CHASSIS_MODE_LITTLE_TOP 小陀螺，独立功能（暂未实现）

        osDelay(1);
    }
}

void Chassis_Init(void)
{
    chassis_cmd = Chassis_Cmd{
        .v = 0.0f,
        .theta = 0.0f,
        .w_rot = 0.0f,
        .yaw_motor_angle = 0.0f,
        .mode = CHASSIS_MODE_NO_ROTATION,
        .enable = 0,
    };

    ChassisMotion::Config cfg = {
        .type = CHASSIS_MOTION_TYPE,
        .r = CHASSIS_WHEEL_RADIUS_M,
        .L = CHASSIS_LENGTH_M,
        .W = CHASSIS_WIDTH_M,
        .velocity = {
            .can_handle = CHASSIS_CAN_HANDLE,
            .motor_id_rf = CHASSIS_MOTOR_ID_RF,
            .motor_id_lf = CHASSIS_MOTOR_ID_LF,
            .motor_id_lr = CHASSIS_MOTOR_ID_LR,
            .motor_id_rr = CHASSIS_MOTOR_ID_RR,
            .reduction_ratio = CHASSIS_REDUCTION_RATIO,
            .pid_velocity = CHASSIS_WHEEL_VELOCITY_PID,
            .power = {
                .torque_constant = CHASSIS_POWER_TORQUE_CONSTANT,
                .speed_loss = CHASSIS_POWER_SPEED_LOSS,
                .torque_loss = CHASSIS_POWER_TORQUE_LOSS,
                .idle_power_w = CHASSIS_POWER_IDLE_W,
                .current_limit_a = (float)djim_current_range[DJIMotor_3508],
            },
        },
        .tim_handle = ROBOT_MOTOR_TIM_HANDLE, // 速度环与功率计算时基；注册顺序由 Robot_Init 决定
    };
    chassis_inst.init(cfg);

    const osThreadAttr_t attr = {
        .name = "Chassis",
        .stack_size = CHASSIS_TASK_STACK_BYTES,
        .priority = osPriorityNormal,
    };
    osThreadNew(Chassis_Task, nullptr, &attr);
}

void Chassis_SetMode(Chassis_Mode mode)
{
    // 切到 NO_ROTATION 时清一次手动角速度，防止从跟随/小陀螺带残留
    if (mode == CHASSIS_MODE_NO_ROTATION && chassis_cmd.mode != CHASSIS_MODE_NO_ROTATION)
        chassis_cmd.w_rot = 0.0f;
    chassis_cmd.mode = mode;
}

float Chassis_GetPower(void)
{
    return chassis_inst.velocity_.getPower();
}
