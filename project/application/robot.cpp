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

    Chassis_Init();
    Gimbal_Init();
    Shoot_Init();

    DJIMotor::timbaseSelect(&htim5);
}

void Robot_Task(void)
{
    Cmd_Task();
}
