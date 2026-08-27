#include "shoot.h"
#include "cmsis_os2.h"

#include "dji_motor.h"

Shoot_Cmd shoot_cmd;

// 电机实例
static DJIMotor_Instance *motor_fric_l;
static DJIMotor_Instance *motor_fric_r;
static DJIMotor_Instance *motor_mag;

// 拨盘状态机
static uint8_t  g_rounds_pending = 0;
static uint16_t g_fire_timer = 0;       // 当前发已过时间 ms
static uint8_t  fire_state = 0;         // 0=IDLE, 1=FIRING

/*
 * TODO: 裁判系统热量检查
 * 若接入裁判系统，在 Shoot_Fire 内调用此函数获取实际允许的发弹量，
 * 强制限弹以避免超热量被罚。
 *
 * uint8_t Referee_GetMaxRounds(uint8_t requested)
 * {
 *     // TODO: 读取裁判系统枪管热量，计算余量
 *     return requested;
 * }
 */

static void Shoot_Task(void *arg);

void Shoot_Init(void)
{
    // 摩擦轮 M3508 (CAN2, ID1:REVERT, ID2:NORMAL)
    DJIMotor_Init_Config_s cfg_fric = {
        .can_handle = &hcan2,
        .motor_type = DJIMotor_3508,
        .reduction_ratio = 19.0f,
        .pid_velocity = {
            .kp = 0.01f,
            .ki = 0.0001f,
            .kd = 0.0f,
            .mode = PID_MODE_DELTA,
            .features = PID_FEATURE_OUTPUT_LIMIT | PID_FEATURE_FILTER,
            .output_min = -20.0f,
            .output_max = 20.0f,
            .filter_alpha = 0.3f,
        },
    };

    cfg_fric.motor_id = 1;
    cfg_fric.direction = DJIM_DIRECTION_REVERT;
    motor_fric_l = DJIMotor_Register(&cfg_fric);

    cfg_fric.motor_id = 2;
    cfg_fric.direction = DJIM_DIRECTION_NORMAL;
    motor_fric_r = DJIMotor_Register(&cfg_fric);

    // 摩擦轮永久开启
    if (motor_fric_l) {
        DJIMotor_Set_Enable(motor_fric_l, 1);
        DJIMotor_Set_Velocity(motor_fric_l, FRIC_SPEED_DEGS);
    }
    if (motor_fric_r) {
        DJIMotor_Set_Enable(motor_fric_r, 1);
        DJIMotor_Set_Velocity(motor_fric_r, FRIC_SPEED_DEGS);
    }

    // 拨盘 M2006 (CAN2, ID3, REVERT, 减速比36)
    DJIMotor_Init_Config_s cfg_mag = {
        .can_handle = &hcan2,
        .motor_type = DJIMotor_2006,
        .motor_id = 3,
        .direction = DJIM_DIRECTION_REVERT,
        .reduction_ratio = 36.0f,
        .pid_angle = {
            .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID_MODE_POSITION,
            .features = PID_FEATURE_OUTPUT_LIMIT,
            .output_min = -360.0f, .output_max = 360.0f,
        },
        .pid_velocity = {
            .kp = 0.01f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID_MODE_DELTA,
            .features = PID_FEATURE_OUTPUT_LIMIT,
            .output_min = -10.0f, .output_max = 10.0f,
        },
    };
    motor_mag = DJIMotor_Register(&cfg_mag);
    if (motor_mag) {
        DJIMotor_Set_Enable(motor_mag, 1);
    }

    // 创建 RTOS 任务
    const osThreadAttr_t attr = {
        .name = "Shoot",
        .stack_size = 512,
        .priority = osPriorityNormal,
    };
    osThreadNew(Shoot_Task, NULL, &attr);
}

void Shoot_Fire(uint8_t count)
{
    // TODO: 裁判系统热量检查
    // if (g_referee_connected) {
    //     count = Referee_GetMaxRounds(count);
    // }
    if (count == 0) return;
    g_rounds_pending += count;
}

static void Shoot_Task(void *arg)
{
    for (;;) {
        DJIMotor_Set_Enable(motor_fric_l, shoot_cmd.enable);
        DJIMotor_Set_Enable(motor_fric_r, shoot_cmd.enable);
        DJIMotor_Set_Enable(motor_mag,    shoot_cmd.enable);

        if (!motor_mag) { osDelay(1); continue; }

        switch (fire_state) {
        case 0: // IDLE
            if (g_rounds_pending > 0) {
                g_fire_timer = 0;
                fire_state = 1;
                DJIMotor_Set_Velocity(motor_mag, MAG_SPEED_DEGS);
            }
            break;

        case 1: // FIRING
            if (g_rounds_pending == 0) {
                DJIMotor_Set_Velocity(motor_mag, 0.0f);
                fire_state = 0;
                break;
            }
            g_fire_timer++;
            if (g_fire_timer >= MAG_MS_PER_ROUND) {
                g_rounds_pending--;
                g_fire_timer = 0;
            }
            break;
        }

        osDelay(1);
    }
}
