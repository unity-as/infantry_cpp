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
#define SHOOT_INIT   !(DEBUG_INIT_MODE && !SHOOT_INIT_DEBUG)// 射击初始化
// ======= 机器人参数 =======

// 云台电机初始位置 — 编码器值
#define GIMBAL_YAW_ECD      6660 - 30 *360 / 8192.f
#define GIMBAL_PITCH_ECD    2710

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
