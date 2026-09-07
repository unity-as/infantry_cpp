#include "robot.h"
#include "config.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "cmd.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

void Robot_Init(void)
{
    DWT_Init();// 初始化 DWT 计数器，用于高精度延时

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

    DJIMotor::timbaseSelect(&htim5);
}

void Robot_Task(void)
{
    Cmd_Task();
}
