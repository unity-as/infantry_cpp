/**
 * @file    chassis.cpp
 * @brief   底盘应用层：读 chassis_cmd，灌入 ChassisMotion（core）
 */
#include "chassis.h"
#include "chassis_motion.h"
#include "cmsis_os2.h"
#include <math.h>

Chassis_Cmd chassis_cmd;
static ChassisMotion chassis_inst;

PID::Config pid_velocity_config =
{
    .kp = 0.05f,
    .ki = 0.0008f,
    .kd = 0.0f,
    .mode = PID::Mode::Position,   // 位置式：反算抗饱和需要可操作的累加积分
    .features = PID::FeatureOutputLimit
              | PID::FeatureIntegralLimit,
    .integral_limit = 20000,
    .output_min = -20.0f,
    .output_max = 20.0f,
    .period = 1,                 // 速度环 1ms
};

static void Chassis_Task(void *arg)
{
    (void)arg;
    for (;;) {
        chassis_inst.velocity_.setEnable(chassis_cmd.enable);
        chassis_inst.v_ = chassis_cmd.v;
        chassis_inst.theta_ = chassis_cmd.theta;

        if (chassis_cmd.mode == CHASSIS_MODE_FOLLOW)
        {
            // float error = fmodf(chassis_cmd.yaw_motor_angle + 180.0f, 360.0f);
            // if (error < 0.0f) error += 360.0f;   // fmodf 结果符号与被除数一致，负数时需归一到 [0,360)
            // error -= 180.0f;
            float error = fmodf(chassis_cmd.yaw_motor_angle + 45.0f, 90.0f);
            if (error < 0.0f) error += 90.0f;   // fmodf 结果符号与被除数一致，负数时需归一到 [0,90)
            error -= 45.0f;
            // chassis_inst.w_rot_ = chassis_inst.theta_ * 2.0f;//跟随底盘移动方向
            chassis_inst.w_rot_ = error * 0.2f;//跟随云台方向

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
        .type = ChassisMotion::Type::Mecanum,
        .r = 0.07625f,
        .L = 0.37f,
        .W = 0.37f,
        .velocity = {
            .can_handle = &hcan1,
            .motor_id_rf = 1,
            .motor_id_lf = 2,
            .motor_id_lr = 3,
            .motor_id_rr = 4,
            .reduction_ratio = DJIM_REDUCTION_RATIO_M3508,
            .pid_velocity = pid_velocity_config,
            .back_calc_coef = 0.07f,    // ≈1/Ti=ki/kp，实现时调
        },
        .tim_handle = &htim5,       // 速度环+功率+发电流跑在此定时器，注册先于 dji_motor
    };
    chassis_inst.init(cfg);

    const osThreadAttr_t attr = {
        .name = "Chassis",
        .stack_size = 256,
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

void Chassis_SetPowerLimit(float limit)
{
    chassis_inst.velocity_.setPowerLimit(limit);
}

float Chassis_GetPower(void)
{
    return chassis_inst.velocity_.getPower();
}
