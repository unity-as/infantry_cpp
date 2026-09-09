#include "robot.h"
#include "config.h"
#include "bsp_dwt.h"
#include "can.h"
#include "tim.h"

#if DM_MOTOR_TEST
#include "dm_motor.h"

static DMMotor dm_test;

#else
#include "dji_motor.h"
#include "cmd.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#endif

void Robot_Init(void)
{
    DWT_Init();// 初始化 DWT 计数器，用于高精度延时

#if DM_MOTOR_TEST
    // 达妙冒烟：只注册一台，使能后力矩 0，靠 TIM 心跳读反馈
    DMMotor::Config cfg{};
    cfg.can_handle = DM_TEST_CAN_HANDLE;
    cfg.control_id = DM_TEST_CONTROL_ID;
    cfg.feedback_id = DM_TEST_FEEDBACK_ID;
    cfg.direction = DM_DIRECTION_NORMAL;
    cfg.reduction_ratio = 1.0f;
    cfg.initial_angle = 0.0f;
    cfg.htim = &htim5;
    cfg.daemon_cycle = 100;

    dm_test.init(cfg);
    dm_test.setEnable(1);
    dm_test.setCurrent(0.0f);
#else
    // 电机失联守护时基：允许先于各子系统 Init（电机注册时补绑）
    DJIMotor::timbaseSelect(&htim5);

    Cmd_Init();// 初始化遥控器、键鼠、AHRS、小电脑通信、裁判系统、RGB灯

    #if GIMBAL_INIT
    Gimbal_Init();
    #endif

    #if CHASSIS_INIT
    Chassis_Init();
    #endif

    #if SHOOT_INIT
    Shoot_Init();
    #endif
#endif
}

void Robot_Task(void)
{
#if DM_MOTOR_TEST
    // 反馈由 dm_motor TIM 回调更新；此处可下断点看：
    // dm_test.angle_ / velocity_ / current_ / motor_valid_ / err_
    (void)dm_test;
#else
    Cmd_Task();
#endif
}
