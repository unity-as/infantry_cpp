#ifndef CHASSIS_VELOCITY_H
#define CHASSIS_VELOCITY_H

#include <math.h>
#include "dji_motor.h"
#include "pid.h"

#define CHASSIS_WHEEL_NUM 4

// Kt(0.3 N·m/A, 输出轴) × π/180；P(W)=系数×I_q(A)×velocity(输出轴deg/s)
#define CHASSIS_POWER_COEF 0.005236f
#define CHASSIS_GET_POWER(motor)  fmaxf(0.0f, CHASSIS_POWER_COEF * DJIM_GET_CURRENT(motor) * DJIM_GET_VELOCITY(motor))

typedef struct {
    CAN_HandleTypeDef *can_handle;//底盘电机CAN句柄
    uint8_t motor_id_rf;//右前轮电机ID
    uint8_t motor_id_lf;//左前轮电机ID
    uint8_t motor_id_lr;//左后轮电机ID
    uint8_t motor_id_rr;//右后轮电机ID

    float reduction_ratio;//减速比

    PID_Init_Config_s pid_velocity;//位置式速度环 PID 配置（四轮共用）
    float back_calc_coef;//反算增益 ≈1/Ti（=ki/kp），实现时调
} ChassisVelocity_Init_Config_s;

typedef struct {
    DJIMotor_Instance *motor[CHASSIS_WHEEL_NUM];    // 四轮电机实例 [rf, lf, lr, rr]
    PID_Instance *pid_velocity[CHASSIS_WHEEL_NUM];  // 四轮外置速度环（位置式）

    float target_omega[CHASSIS_WHEEL_NUM];          // 本周期目标速度 (deg/s, 输出轴)
    float current_raw[CHASSIS_WHEEL_NUM];           // 本周期速度环输出 I_raw (A)
    float back_calc_coef;                           // 反算增益

    float power_limit;//功率上限 (W)：<=0 不限流
    float power_total;//实测总功率 (W)，供监测
    float scale;//当前缩放比 k（1=未限流）

    uint8_t enable;//使能：0=速度环不跑、PID 复位、输出 0 电流
} ChassisVelocity_Instance;

ChassisVelocity_Instance *ChassisVelocity_Register(ChassisVelocity_Init_Config_s *config);

// 收目标速度指令（deg/s，输出轴），由 motion 派发
void ChassisVelocity_SetTarget(ChassisVelocity_Instance *instance, const float omega_deg[CHASSIS_WHEEL_NUM]);

// 跑完整链：速度环 → 预测功率 → 缩放 → 限幅电流 → 反算抗饱和 → 发电流
void ChassisVelocity_Tick(ChassisVelocity_Instance *instance);

void ChassisVelocity_SetEnable(ChassisVelocity_Instance *instance, uint8_t enable);
void ChassisVelocity_SetPowerLimit(ChassisVelocity_Instance *instance, float limit);
float ChassisVelocity_GetPower(ChassisVelocity_Instance *instance);

#endif
