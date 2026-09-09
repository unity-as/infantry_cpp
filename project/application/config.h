#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// ======= 调试选项 =======

// 达妙单电机冒烟：1=关闭 cmd/底盘/云台/射击，只在 robot 里注册一台 DM
#define DM_MOTOR_TEST 1

#if DM_MOTOR_TEST
#define DM_TEST_CAN_HANDLE   (&hcan1)   // 接哪路 CAN 改这里（&hcan1 / &hcan2）
#define DM_TEST_CONTROL_ID   0x01u      // 助手 CANID（控制基 ID）
#define DM_TEST_FEEDBACK_ID  0x11u      // 助手 MasterID（反馈 ID）
#endif

// 开启任意项，可以单独启动不同的模块，方便调试（DM_MOTOR_TEST=1 时无效）
#define GIMBAL_INIT_DEBUG 0
#define CHASSIS_INIT_DEBUG 0
#define SHOOT_INIT_DEBUG 0

#if DM_MOTOR_TEST
#define GIMBAL_INIT  0
#define CHASSIS_INIT 0
#define SHOOT_INIT   0
#else
#if (GIMBAL_INIT_DEBUG || CHASSIS_INIT_DEBUG || SHOOT_INIT_DEBUG)
#define DEBUG_INIT_MODE 1
#else
#define DEBUG_INIT_MODE 0
#endif

#define GIMBAL_INIT  !(DEBUG_INIT_MODE && !GIMBAL_INIT_DEBUG)// 云台初始化
#define CHASSIS_INIT !(DEBUG_INIT_MODE && !CHASSIS_INIT_DEBUG) // 底盘初始化
#define SHOOT_INIT   !(DEBUG_INIT_MODE && !SHOOT_INIT_DEBUG)// 射击初始化
#endif
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
