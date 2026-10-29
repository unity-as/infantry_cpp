#pragma once

// 整车可调参数唯一入口；应用层组装各模块 Config，下层不依赖本文件。
// ======= 子系统初始化（各项独立：0=跳过，1=初始化）=======
#define GIMBAL_INIT  1
#define CHASSIS_INIT 1
#define SHOOT_INIT   0  // 整个发射机构：摩擦轮 + 拨盘 + Shoot_Task

// ======= 板级绑定与任务资源 =======
#define ROBOT_MOTOR_TIM_HANDLE (&htim5)  // 当前硬件周期 1 ms；修改时同步核对 PID 周期
#define CHASSIS_CAN_HANDLE     (&hcan1)
#define GIMBAL_CAN_HANDLE      (&hcan2)
#define SHOOT_CAN_HANDLE       (&hcan2)
#define VT13_UART_HANDLE       (&huart3)
#define MINIPC_UART_HANDLE     (&huart6)
#define REFEREE_UART_HANDLE    (&huart1)
#define POWER_ADC_HANDLE       (&hadc3)
#define CHASSIS_TASK_STACK_BYTES 256
#define GIMBAL_TASK_STACK_BYTES  1024
#define SHOOT_TASK_STACK_BYTES   512

// ======= AHRS 安装与采样 =======
#define AHRS_ACCEL_RANGE       BMI088::AccRange::G6
#define AHRS_GYRO_RANGE        BMI088::GyroRange::Dps2000
#define AHRS_BOARD_YAW_DEG     180.0f
#define AHRS_BOARD_PITCH_DEG   0.0f
#define AHRS_BOARD_ROLL_DEG    -90.0f
#define AHRS_ACCEL_LPF_COEF    0.0085f

// ======= 操作手感 =======
#define REMOTE_GIMBAL_YAW_MAX_DEG_S   200.0f
#define REMOTE_GIMBAL_PITCH_MAX_DEG_S 100.0f
#define REMOTE_CHASSIS_V_MAX_MPS      1.5f
#define REMOTE_CHASSIS_W_MAX_REV      1.5f  // 圈/s
#define KEYBOARD_CHASSIS_V_MPS        1.5f
#define MOUSE_GIMBAL_YAW_SCALE        0.001f
#define MOUSE_GIMBAL_PITCH_SCALE      0.0005f
#define CMD_TELEMETRY_PERIOD_CYCLES   10    // Cmd_Task 调用次数
#define CMD_STARTUP_DELAY_CYCLES      1000  // Cmd_Task 调用次数

// ======= 底盘几何、接线与跟随 =======
#define CHASSIS_MOTION_TYPE       ChassisMotion::Type::Mecanum
#define CHASSIS_WHEEL_RADIUS_M    0.07625f
#define CHASSIS_LENGTH_M          0.37f
#define CHASSIS_WIDTH_M           0.37f
#define CHASSIS_MOTOR_ID_RF       1
#define CHASSIS_MOTOR_ID_LF       2
#define CHASSIS_MOTOR_ID_LR       3
#define CHASSIS_MOTOR_ID_RR       4
#define CHASSIS_REDUCTION_RATIO   DJIM_REDUCTION_RATIO_M3508
#define CHASSIS_FOLLOW_PERIOD_DEG 90.0f
#define CHASSIS_FOLLOW_GAIN       0.2f  // yaw 误差 deg → 底盘角速度 rad/s
#define CHASSIS_BACK_CALC_COEF    0.00007f  // 每拍电流差 → 积分回灌量（误差·s/A）

// ======= 底盘缓冲能量与功率决策 =======
#define CHASSIS_BUFFER_MAX_J       60.0f  // RGB 归一化与功率计算共用
#define CHASSIS_BUFFER_RELEASE_J   40.0f  // 大于该值时放开功率限制
#define CHASSIS_POWER_MARGIN_W     10

// ======= 云台零位、限位与前馈 =======
#define GIMBAL_MOTOR_ID_YAW    1
#define GIMBAL_MOTOR_ID_PITCH  2
#define GIMBAL_YAW_ECD        (60*8192.0f/360.0f)
#define GIMBAL_PITCH_ECD      1948.0f  // 底盘水平、AHRS pitch=0 时的原始 ECD
#define GIMBAL_YAW_AHRS_MIN     0.0f  // min=max 时不启用 yaw 角限幅
#define GIMBAL_YAW_AHRS_MAX     0.0f
// AHRS 为姿态角，MOTOR 为相对上述编码器零位的电机角，单位 deg。
#define GIMBAL_PITCH_AHRS_MIN  -30.7f
#define GIMBAL_PITCH_AHRS_MAX   10.5f
#define GIMBAL_PITCH_MOTOR_MIN -30.7f
#define GIMBAL_PITCH_MOTOR_MAX  10.5f
#define GIMBAL_YAW_CURRENT_FF    0.0f
#define GIMBAL_PITCH_CURRENT_FF -0.150f
#define GIMBAL_POS_FREQ_DIV_YAW   5  // 与对应位置环 PID 的 period 配套调整
#define GIMBAL_POS_FREQ_DIV_PITCH 5
#define GIMBAL_AIM_FREQ_DIV       10

// ======= 发射机构接线与动作 =======
#define SHOOT_FRIC_MOTOR_TYPE      DJIMotor_3508
#define SHOOT_FRIC_LEFT_MOTOR_ID   1
#define SHOOT_FRIC_RIGHT_MOTOR_ID  2
#define SHOOT_FRIC_LEFT_DIRECTION  DJIM_DIRECTION_REVERT
#define SHOOT_FRIC_RIGHT_DIRECTION DJIM_DIRECTION_NORMAL
#define SHOOT_FRIC_REDUCTION_RATIO 19.0f
#define SHOOT_MAG_MOTOR_TYPE       DJIMotor_2006
#define SHOOT_MAG_MOTOR_ID         3
#define SHOOT_MAG_DIRECTION        DJIM_DIRECTION_REVERT
#define SHOOT_MAG_REDUCTION_RATIO  36.0f
#define FRIC_SPEED_DEGS            2500.0f  // 输出轴 deg/s
#define MAG_CAPACITY               8        // 发/圈
#define SHOOT_PERIOD_MS            64

#ifdef __cplusplus
#include "pid.h"

// ======= PID 完整配置（period 单位 s，未指定字段为 0）=======

// 底盘四轮速度环（位置式，供功率限流反算积分）
inline constexpr PID::Config CHASSIS_WHEEL_VELOCITY_PID = {
    .kp = 0.0375f,
    .ki = 0.3f,
    .kd = 0.0f,
    .mode = PID::Mode::Position,   // 位置式：反算抗饱和需要可操作的累加积分
    .features = PID::FeatureOutputLimit
              | PID::FeatureIntegralLimit,
    .integral_limit = 40.0f,
    .output_min = -20.0f,
    .output_max = 20.0f,
    .period = 0.001f,            // TIM5 1 ms
};

// 云台 yaw 位置环
inline constexpr PID::Config GIMBAL_YAW_POSITION_PID = {
    .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
    .mode = PID::Mode::Position,
    .features = PID::FeatureOutputLimit,
    .output_min = -480.0f, .output_max = 480.0f,
    .period = 0.005f,  // TIM5 1 ms × 位置环 5 分频
};

// 云台 pitch 位置环
inline constexpr PID::Config GIMBAL_PITCH_POSITION_PID = {
    .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
    .mode = PID::Mode::Position,
    .features = PID::FeatureOutputLimit
              | PID::FeatureIntegralLimit,
    .integral_limit = 0.0f,
    .output_min = -240.0f, .output_max = 240.0f,
    .filter_alpha = 0.3f,
    .period = 0.005f,  // TIM5 1 ms × 位置环 5 分频
};

// 云台 yaw 速度环
inline constexpr PID::Config GIMBAL_YAW_VELOCITY_PID = {
    .kp = 0.04f, .ki = 0.25f, .kd = 0.00004f,
    .mode = PID::Mode::Position,
    .features = PID::FeatureOutputLimit
              | PID::FeatureIntegralLimit
              | PID::FeatureDerivativeOnMeasurement
              | PID::FeatureFilter,
    .integral_limit = 0.4f,
    .output_min = -1.0f, .output_max = 1.0f,
    .filter_alpha = 0.10557281f,  // 1 - sqrt(1 - 0.2)，保持原 2 ms 滤波时间尺度
    .kp_extra = 0.00001f,
    .period = 0.001f,
};

// 云台 pitch 速度环
inline constexpr PID::Config GIMBAL_PITCH_VELOCITY_PID = {
    .kp = 0.007f, .ki = 0.03f, .kd = 0.000004f,
    .mode = PID::Mode::Position,
    .features = PID::FeatureOutputLimit
              | PID::FeatureIntegralLimit
              | PID::FeatureDerivativeOnMeasurement
              | PID::FeatureFilter,
    .integral_limit = 20.0f,
    .output_min = -1.0f, .output_max = 1.0f,
    .filter_alpha = 0.29289322f,  // 1 - sqrt(1 - 0.5)，保持原 2 ms 滤波时间尺度
    .period = 0.001f,
};

// 自瞄 yaw 误差 → 速度
inline constexpr PID::Config GIMBAL_AIM_YAW_PID = {
    .kp = 6.0f, .ki = 0.0f, .kd = 0.00001f,
    .mode = PID::Mode::Position,
    .features = PID::FeatureOutputLimit
              | PID::FeatureIntegralLimit,
    .integral_limit = 0.18f,
    .output_min = -360.0f, .output_max = 360.0f,
    .filter_alpha = 0.5f,
    .period = 0.001f,
};

// 自瞄 pitch 误差 → 速度
inline constexpr PID::Config GIMBAL_AIM_PITCH_PID = {
    .kp = 8.0f, .ki = 0.0f, .kd = 0.00001f,
    .mode = PID::Mode::Position,
    .features = PID::FeatureOutputLimit
              | PID::FeatureIntegralLimit,
    .integral_limit = 0.18f,
    .output_min = -360.0f, .output_max = 360.0f,
    .filter_alpha = 0.5f,
    .period = 0.001f,
};

// 摩擦轮速度环
inline constexpr PID::Config SHOOT_FRIC_VELOCITY_PID = {
    .kp = 0.01f,
    .ki = 0.05f,
    .kd = 0.0f,
    .mode = PID::Mode::Delta,
    .features = PID::FeatureOutputLimit | PID::FeatureFilter,
    .output_min = -20.0f,
    .output_max = 20.0f,
    .filter_alpha = 0.16333997f,  // 1 - sqrt(1 - 0.3)，对应 TIM5 1 ms
    .period = 0.001f,
};

// 拨盘位置环
inline constexpr PID::Config SHOOT_MAG_POSITION_PID = {
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .mode = PID::Mode::Position,
    .features = PID::FeatureOutputLimit,
    .output_min = -360.0f, .output_max = 360.0f,
    .period = 0.001f,
};

// 拨盘速度环
inline constexpr PID::Config SHOOT_MAG_VELOCITY_PID = {
    .kp = 0.01f, .ki = 0.0f, .kd = 0.0f,
    .mode = PID::Mode::Delta,
    .features = PID::FeatureOutputLimit,
    .output_min = -10.0f, .output_max = 10.0f,
    .period = 0.001f,
};

#endif // __cplusplus
