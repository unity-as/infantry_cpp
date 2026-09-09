/**
 * @file    shoot.cpp
 * @brief   射击模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 shoot 迁移，逻辑不变，禁堆（电机实例由指针改为全局对象）。
 *          Shoot_SetLoader 供 DT7 拨轮连发/反转；与 Shoot_Fire 计数供弹互斥。
 */
#include "shoot.h"
#include "cmsis_os2.h"

#include "dji_motor.h"

Shoot_Cmd shoot_cmd;

// 电机实例
static DJIMotor motor_fric_l;
static DJIMotor motor_fric_r;
static DJIMotor motor_mag;

// 拨盘状态机（Shoot_Fire）
static uint8_t  g_rounds_pending = 0;
static uint16_t g_fire_timer = 0;       // 当前发已过时间 ms
static uint8_t  fire_state = 0;         // 0=IDLE, 1=FIRING

// 拨盘连续模式（Shoot_SetLoader，优先于计数供弹）
static volatile Shoot_LoaderMode g_loader_mode = SHOOT_LOADER_STOP;

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
    DJIMotor::Config cfg_fric = {
        .can_handle = &hcan2,
        .motor_type = DJIMotor_3508,
        .reduction_ratio = 19.0f,
        .pid_velocity = {
            .kp = 0.01f,
            .ki = 0.0001f,
            .kd = 0.0f,
            .mode = PID::Mode::Delta,
            .features = PID::FeatureOutputLimit | PID::FeatureFilter,
            .output_min = -20.0f,
            .output_max = 20.0f,
            .filter_alpha = 0.3f,
        },
    };

    cfg_fric.motor_id = 1;
    cfg_fric.direction = DJIM_DIRECTION_REVERT;
    motor_fric_l.init(cfg_fric);

    cfg_fric.motor_id = 2;
    cfg_fric.direction = DJIM_DIRECTION_NORMAL;
    motor_fric_r.init(cfg_fric);

    // 摩擦轮永久开启
    motor_fric_l.setEnable(1);
    motor_fric_l.setVelocity(FRIC_SPEED_DEGS);
    motor_fric_r.setEnable(1);
    motor_fric_r.setVelocity(FRIC_SPEED_DEGS);

    // 拨盘 M2006 (CAN2, ID3, REVERT, 减速比36)
    DJIMotor::Config cfg_mag = {
        .can_handle = &hcan2,
        .motor_id = 3,
        .motor_type = DJIMotor_2006,
        .direction = DJIM_DIRECTION_REVERT,
        .reduction_ratio = 36.0f,
        .pid_angle = {
            .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID::Mode::Position,
            .features = PID::FeatureOutputLimit,
            .output_min = -360.0f, .output_max = 360.0f,
        },
        .pid_velocity = {
            .kp = 0.01f, .ki = 0.0f, .kd = 0.0f,
            .mode = PID::Mode::Delta,
            .features = PID::FeatureOutputLimit,
            .output_min = -10.0f, .output_max = 10.0f,
        },
    };
    motor_mag.init(cfg_mag);
    motor_mag.setEnable(1);

    // 创建 RTOS 任务
    const osThreadAttr_t attr = {
        .name = "Shoot",
        .stack_size = 512,
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
    if (g_loader_mode != SHOOT_LOADER_STOP) return;
    g_rounds_pending += count;
}

void Shoot_SetLoader(Shoot_LoaderMode mode)
{
    g_loader_mode = mode;
    if (mode != SHOOT_LOADER_STOP) {
        g_rounds_pending = 0;
        g_fire_timer = 0;
        fire_state = 0;
    }
}

static void Shoot_Task(void *arg)
{
    for (;;) {
        motor_fric_l.setEnable(shoot_cmd.enable);
        motor_fric_r.setEnable(shoot_cmd.enable);
        motor_mag.setEnable(shoot_cmd.enable);

        Shoot_LoaderMode loader = g_loader_mode;
        if (loader != SHOOT_LOADER_STOP) {
            if (loader == SHOOT_LOADER_BURST)
                motor_mag.setVelocity(MAG_SPEED_DEGS);
            else
                motor_mag.setVelocity(-MAG_SPEED_DEGS);
            osDelay(1);
            continue;
        }

        switch (fire_state) {
        case 0: // IDLE
            if (g_rounds_pending > 0) {
                g_fire_timer = 0;
                fire_state = 1;
                motor_mag.setVelocity(MAG_SPEED_DEGS);
            } else {
                motor_mag.setVelocity(0.0f);
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
