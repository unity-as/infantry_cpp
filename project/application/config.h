#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// ======= 调试选项 =======

// 开启任意项，可以单独启动不同的模块，方便调试
#define GIMBAL_INIT_DEBUG 0
#define CHASSIS_INIT_DEBUG 0
#define SHOOT_INIT_DEBUG 0

#if (GIMBAL_INIT_DEBUG || CHASSIS_INIT_DEBUG || SHOOT_INIT_DEBUG)
#define DEBUG_INIT_MODE 1
#else
#define DEBUG_INIT_MODE 0
#endif

#define GIMBAL_INIT  !(DEBUG_INIT_MODE && !GIMBAL_INIT_DEBUG)// 云台初始化
#define CHASSIS_INIT !(DEBUG_INIT_MODE && !CHASSIS_INIT_DEBUG) // 底盘初始化
#define SHOOT_INIT   0 // 发射机构初始化：0=关闭，1=开启（摩擦轮和拨盘一起控制）
// ======= 机器人参数 =======

// 云台电机初始位置 — 编码器值
#define GIMBAL_YAW_ECD      (60*8192.0f/360.0f)  // 云台偏航电机初始位置 (编码器值)
#define GIMBAL_PITCH_ECD    1948.0f  // 底盘水平、AHRS pitch=0 时的原始编码器值

// pitch 限位 [deg]：AHRS 为姿态角，MOTOR 为相对上述编码器零位的电机角
#define GIMBAL_PITCH_AHRS_MIN   -30.7f
#define GIMBAL_PITCH_AHRS_MAX    10.5f
#define GIMBAL_PITCH_MOTOR_MIN  -30.7f
#define GIMBAL_PITCH_MOTOR_MAX   10.5f

// 云台重力补偿torque
#define GIMBAL_PITCH_CURRENT_FF  -0.150f

// 发射参数
#define FRIC_SPEED_DEGS       2500.0f   // 摩擦轮工作速度 (输出轴 deg/s)
#define MAG_CAPACITY             8       // 拨盘装载量 发/圈
#define SHOOT_PERIOD_MS           64     // 发弹周期 (ms)


#ifdef __cplusplus
}
#endif

#endif
