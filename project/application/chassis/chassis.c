#include "chassis.h"
#include "cmsis_os2.h"
#include <math.h>

Chassis_Cmd chassis_cmd;
ChassisMotion_Instance *chassis_inst = NULL;

PID_Init_Config_s pid_velocity_config =
{
    .kp = 0.05f,
    .ki = 0.0008f,
    .kd = 0.0f,
    .mode = PID_MODE_POSITION,   // 位置式：反算抗饱和需要可操作的累加积分
    .features = PID_FEATURE_OUTPUT_LIMIT
    | PID_FEATURE_INTEGRAL_LIMIT,
    .integral_limit = 20000,
    .output_min = -20.0f,
    .output_max = 20.0f,
    .period = 1,                 // 速度环 1ms
};

static void Chassis_Task(void *arg)
{
    (void)arg;
    for (;;) {
        ChassisVelocity_SetEnable(chassis_inst->velocity, chassis_cmd.enable);

        if (chassis_cmd.mode == CHASSIS_MODE_FOLLOW)
        {
            float error = fmodf(chassis_cmd.yaw_motor_angle + 180.0f, 360.0f);
            if (error < 0.0f) error += 360.0f;   // fmodf 结果符号与被除数一致，负数时需归一到 [0,360)
            error -= 180.0f;
            // chassis_inst->w_rot = chassis_inst->theta * 2.0f;//跟随底盘移动方向
            chassis_inst->w_rot = error * 4 *3.14/180.0f;//跟随云台方向

            //纯P就懒得上pid了
        }
        else if (chassis_cmd.mode == CHASSIS_MODE_NO_ROTATION)
            chassis_inst->w_rot = chassis_cmd.w_rot;//手动角速度（拨轮）
        else
            chassis_inst->w_rot = 0.0f;//CHASSIS_MODE_LITTLE_TOP 小陀螺，独立功能（暂未实现）

        osDelay(1);
    }
}

void Chassis_Init(void)
{
    chassis_cmd = (Chassis_Cmd){
        .v = 0.0f,
        .w_rot = 0.0f,
        .mode = CHASSIS_MODE_NO_ROTATION,
    };

    ChassisMotion_Init_Config_s cfg = {
        .type = CHASSIS_TYPE_MECANUM,
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
    chassis_inst = ChassisMotion_Register(&cfg);

    const osThreadAttr_t attr = {
        .name = "Chassis",
        .stack_size = 256,
        .priority = osPriorityNormal,
    };
    osThreadNew(Chassis_Task, NULL, &attr);
}

void Chassis_SetMode(Chassis_Mode mode)
{
    // 切到 NO_ROTATION 时清一次手动角速度，防止从跟随/小陀螺带残留
    if (mode == CHASSIS_MODE_NO_ROTATION && chassis_cmd.mode != CHASSIS_MODE_NO_ROTATION)
        chassis_cmd.w_rot = 0.0f;
    chassis_cmd.mode = mode;
}

void Chassis_SetPowerLimit(ChassisMotion_Instance *chassis, float limit)
{
    if (chassis && chassis->velocity)
        ChassisVelocity_SetPowerLimit(chassis->velocity, limit);
}

float Chassis_GetPower(ChassisMotion_Instance *chassis)
{
    if (!chassis) return 0.0f;
    return ChassisVelocity_GetPower(chassis->velocity);
}
