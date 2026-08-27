#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include "bsp_tim.h"
#include "chassis_velocity.h"

typedef enum {
    CHASSIS_TYPE_MECANUM,//麦克纳姆底盘
    CHASSIS_TYPE_OMNI,//全向轮底盘
} Chassis_Type;

typedef struct {
    Chassis_Type type;
    float r;//轮半径
    float D;//底盘上的轮心间距
    float L;//轮子纵向轴距
    float W;//轮子横向间距

    ChassisVelocity_Init_Config_s velocity;//[2] 速度环+功率（四轮电机）
    TIM_HandleTypeDef *tim_handle;//1kHz 定时器（注册先于 dji_motor）
} ChassisMotion_Init_Config_s;

typedef struct {
    /* 运动学参数 */
    float move_scale;//移动缩放因子
    float rotate_scale;//旋转缩放因子

    /* 底盘输入量 */
    float theta;//底盘朝向角度，单位弧度
    float v;//底盘移动速度，单位m/s
    float w_rot;//底盘旋转速度，单位rad/s

    /* 下层模块 */
    ChassisVelocity_Instance *velocity;//[2] 速度环+功率
} ChassisMotion_Instance;

ChassisMotion_Instance *ChassisMotion_Register(ChassisMotion_Init_Config_s *config);

#endif
