/**
 * @file    shoot.cpp
 * @brief   射击模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 shoot 迁移，逻辑不变，禁堆（电机实例由指针改为全局对象）。
 */
#include "shoot.h"
#include "cmsis_os2.h"

#include "dji_motor.h"

Shoot_Cmd shoot_cmd;

// 电机实例
static DJIMotor motor_fric_l;
static DJIMotor motor_fric_r;
static DJIMotor motor_mag;

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
    // 摩擦轮接线、方向与 PID 从整车配置注入。
    DJIMotor::Config cfg_fric = {
        .can_handle = SHOOT_CAN_HANDLE,
        .motor_type = SHOOT_FRIC_MOTOR_TYPE,
        .reduction_ratio = SHOOT_FRIC_REDUCTION_RATIO,
        .pid_velocity = SHOOT_FRIC_VELOCITY_PID,
    };

    cfg_fric.motor_id = SHOOT_FRIC_LEFT_MOTOR_ID;
    cfg_fric.direction = SHOOT_FRIC_LEFT_DIRECTION;
    motor_fric_l.init(cfg_fric);

    cfg_fric.motor_id = SHOOT_FRIC_RIGHT_MOTOR_ID;
    cfg_fric.direction = SHOOT_FRIC_RIGHT_DIRECTION;
    motor_fric_r.init(cfg_fric);

    // 摩擦轮使能跟随射击指令，机构是否初始化由 SHOOT_INIT 控制。
    motor_fric_l.setEnable(shoot_cmd.enable);
    motor_fric_l.setVelocity(FRIC_SPEED_DEGS);
    motor_fric_r.setEnable(shoot_cmd.enable);
    motor_fric_r.setVelocity(FRIC_SPEED_DEGS);

    // 拨盘
    DJIMotor::Config cfg_mag = {
        .can_handle = SHOOT_CAN_HANDLE,
        .motor_id = SHOOT_MAG_MOTOR_ID,
        .motor_type = SHOOT_MAG_MOTOR_TYPE,
        .direction = SHOOT_MAG_DIRECTION,
        .reduction_ratio = SHOOT_MAG_REDUCTION_RATIO,
        .pid_angle = SHOOT_MAG_POSITION_PID,
        .pid_velocity = SHOOT_MAG_VELOCITY_PID,
    };
    motor_mag.init(cfg_mag);
    motor_mag.setEnable(1);

    // 创建 RTOS 任务
    const osThreadAttr_t attr = {
        .name = "Shoot",
        .stack_size = SHOOT_TASK_STACK_BYTES,
        .priority = osPriorityNormal,
    };
    osThreadNew(Shoot_Task, nullptr, &attr);
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
        motor_fric_l.setEnable(shoot_cmd.enable);
        motor_fric_r.setEnable(shoot_cmd.enable);
        motor_mag.setEnable(shoot_cmd.enable);

        switch (fire_state) {
        case 0: // IDLE
            if (g_rounds_pending > 0) {
                g_fire_timer = 0;
                fire_state = 1;
                motor_mag.setVelocity(MAG_SPEED_DEGS);
            }
            break;

        case 1: // FIRING
            if (g_rounds_pending == 0) {
                motor_mag.setVelocity(0.0f);
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
