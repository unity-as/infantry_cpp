#ifndef AHRS_H
#define AHRS_H

#include "bmi088.h"
#include "kalman_filter.h"
#include "bsp_dwt.h"
#include "imu_temp.h"

/*============================================
 * 功能开关
 * AHRS_USE_MAGNETOMETER: 启用/禁用磁力计融合
 *   0 = 仅使用陀螺仪+加速度计 (俯仰/横滚修正)
 *   1 = 使用陀螺仪+加速度计+磁力计 (偏航也修正)
 ============================================*/

#ifndef AHRS_USE_MAGNETOMETER
#define AHRS_USE_MAGNETOMETER    0
#endif

#if AHRS_USE_MAGNETOMETER
#include "ist8310.h"
#endif

/*============================================
 * 校验开关
 * AHRS_ENABLE_VALIDATION: 启用/禁用 EKF 校验
 *   0 = 不校验（默认，简洁快速）
 *   1 = 启用模长校验、协方差边界检查等
 ============================================*/

#ifndef AHRS_ENABLE_VALIDATION
#define AHRS_ENABLE_VALIDATION    1
#endif

/*============================================
 * 动态依赖切换
 ============================================*/

#if defined(AHRS_DISABLE_RTOS)
    #define AHRS_RTOS_SUPPORT 0
#elif __has_include("cmsis_os2.h")
    #include "cmsis_os2.h"
    #define AHRS_RTOS_SUPPORT 2
#elif __has_include("cmsis_os.h")
    #include "cmsis_os.h"
    #define AHRS_RTOS_SUPPORT 1
#else
    #define AHRS_RTOS_SUPPORT 0
#endif

/*============================================
 * 状态维度定义
 *
 * 无磁力计: n=6 (q0,q1,q2,q3, bx,by)
 * 有磁力计: n=7 (q0,q1,q2,q3, bx,by, bz)
 *
 * 观测维度定义
 * 无磁力计: m=3 (ax,ay,az)
 * 有磁力计: m=6 (ax,ay,az, mx,my,mz)
 ============================================*/

#if AHRS_USE_MAGNETOMETER
    #define AHRS_STATE_DIM     7
    #define AHRS_OBS_DIM       6
#else
    #define AHRS_STATE_DIM     6
    #define AHRS_OBS_DIM       3
#endif

/*============================================
 * 温控周期
 ============================================*/

#define AHRS_TEMP_CTRL_PERIOD_MS    750     // 温控采样周期 [ms]

/*============================================
 * 输出结构体
 ============================================*/

typedef struct {
    // 姿态
    float q[4];                 // 四元数 (q0, q1, q2, q3)
    float euler[3];             // 欧拉角 (roll, pitch, yaw) [deg]
    float yaw_total;            // 累计偏航角 [-inf, +inf] deg

    // 角速度（机体坐标系，安装角旋转+零偏去除后）[rad/s]
    float gyro_b[3];            // (x, y, z)

    // 加速度（机体坐标系，安装角旋转+低通滤波后，含重力）[m/s²]
    float accel_b[3];           // (x, y, z)

#if AHRS_USE_MAGNETOMETER
    // 磁力计（机体坐标系，安装角旋转后）
    float mag_b[3];             // (x, y, z) [原始单位]
#endif

    // 运动加速度（去重力）[m/s²]
    float motion_accel_b[3];    // 机体坐标系
    float motion_accel_n[3];    // 世界坐标系
} AHRS_Output_t;

/*============================================
 * AHRS 实例结构体
 ============================================*/

typedef struct {
    AHRS_Output_t output;

    // 传感器驱动
    BMI088_Instance *bmi088;
#if AHRS_USE_MAGNETOMETER
    IST8310_Instance *ist8310;
#endif

    // 卡尔曼滤波器
    KF_Instance *kf;

    // 温度控制
    IMU_Temp_Instance *imu_temp;

    // 安装角旋转矩阵 (B→I)
    float install_rot[9];      // 3×3 旋转矩阵

    // 加速度低通滤波
    float accel_lpf_coef;       // 滤波系数 [s]

    // 陀螺仪零偏
    float gyro_bias[3];        // 角速度零偏 [rad/s]

    // 欧拉角累计
    uint32_t dwt_cnt;          // DWT计数器
    int16_t yaw_round_count;   // 偏航角圈数计数
    float yaw_last;            // 上次偏航角 [deg]

    // EKF 噪声配置
    float process_noise_quat;    // Q 系数：四元数过程噪声
    float process_noise_bias;    // Q 系数：零偏随机游走
    float obs_noise_accel;       // R 系数：加速度计观测噪声
#if AHRS_USE_MAGNETOMETER
    float obs_noise_mag;         // R 系数：磁力计观测噪声
#endif

    // 预热配置
    uint32_t preheat_timeout_ms; // 预热超时 [ms]

    // EKF 工作缓冲区
    float Q_data[AHRS_STATE_DIM * AHRS_STATE_DIM];
    float R_accel[9];             // 加速度计观测噪声矩阵 (3×3)
#if AHRS_USE_MAGNETOMETER
    float R_mag[9];               // 磁力计观测噪声矩阵 (3×3)
#endif
} AHRS_Instance;

/*============================================
 * 初始化配置
 ============================================*/

typedef struct {
    BMI088_AccRange_e accel_range;
    BMI088_GyroRange_e gyro_range;
#if AHRS_USE_MAGNETOMETER
    IST8310_Init_Config_s ist8310_config;
#endif
    float board_yaw;         // 板子相对云台的偏航角 [deg]（正值=板子逆时针转）
    float board_pitch;       // 板子相对云台的俯仰角 [deg]（正值=板子低头）
    float board_roll;        // 板子相对云台的横滚角 [deg]（正值=板子右倾）
    float accel_lpf_coef;     // 加速度低通滤波系数 [s]

    // EKF 噪声配置 (<=0 时使用默认值)
    float process_noise_quat;    // Q: 四元数过程噪声系数，默认 10.0f
    float process_noise_bias;    // Q: 零偏随机游走系数，默认 0.001f
    float obs_noise_accel;       // R: 加速度计观测噪声，默认 1000000.0f
#if AHRS_USE_MAGNETOMETER
    float obs_noise_mag;         // R: 磁力计观测噪声，默认 1000000.0f
#endif

    // 预热配置 (<=0 时使用默认值)
    uint32_t preheat_timeout_ms; // 预热超时 [ms]，默认 30000
} AHRS_Init_Config_s;

/*============================================
 * API
 ============================================*/

AHRS_Instance *AHRS_Register(AHRS_Init_Config_s *config);
void AHRS_Preheat(AHRS_Instance *ahrs);
void AHRS_Calibrate(AHRS_Instance *ahrs);
void AHRS_Start(AHRS_Instance *ahrs);
void AHRS_Free(AHRS_Instance *ahrs);

#if !AHRS_RTOS_SUPPORT
void AHRS_Task(void);
#endif

/*============================================
 * 数据访问宏
 *
 * 角速度: [rad/s] (机体坐标系，纠偏后)
 * 加速度: [m/s²] (机体坐标系，纠偏后，含重力)
 * 欧拉角: [deg]
 * 温度:   [°C]
 ============================================*/

// 角速度 [rad/s]
#define AHRS_GetGyroX(ahrs)     ((ahrs)->output.gyro_b[0])
#define AHRS_GetGyroY(ahrs)     ((ahrs)->output.gyro_b[1])
#define AHRS_GetGyroZ(ahrs)     ((ahrs)->output.gyro_b[2])

// 加速度 [m/s²]
#define AHRS_GetAccelX(ahrs)    ((ahrs)->output.accel_b[0])
#define AHRS_GetAccelY(ahrs)    ((ahrs)->output.accel_b[1])
#define AHRS_GetAccelZ(ahrs)    ((ahrs)->output.accel_b[2])

// 温度 [°C]
#define AHRS_GetTemperature(ahrs) ((ahrs)->bmi088->temperature)

// 四元数（无单位）
#define AHRS_GetQ0(ahrs)        ((ahrs)->output.q[0])
#define AHRS_GetQ1(ahrs)        ((ahrs)->output.q[1])
#define AHRS_GetQ2(ahrs)        ((ahrs)->output.q[2])
#define AHRS_GetQ3(ahrs)        ((ahrs)->output.q[3])

// 欧拉角 [deg]
#define AHRS_GetRoll(ahrs)      ((ahrs)->output.euler[0])
#define AHRS_GetPitch(ahrs)     ((ahrs)->output.euler[1])
#define AHRS_GetYaw(ahrs)       ((ahrs)->output.euler[2])
#define AHRS_GetYawTotal(ahrs)  ((ahrs)->output.yaw_total)

// 运动加速度 [m/s²]（去重力）
#define AHRS_GetMotionAccelBX(ahrs) ((ahrs)->output.motion_accel_b[0])
#define AHRS_GetMotionAccelBY(ahrs) ((ahrs)->output.motion_accel_b[1])
#define AHRS_GetMotionAccelBZ(ahrs) ((ahrs)->output.motion_accel_b[2])

#define AHRS_GetMotionAccelNX(ahrs) ((ahrs)->output.motion_accel_n[0])
#define AHRS_GetMotionAccelNY(ahrs) ((ahrs)->output.motion_accel_n[1])
#define AHRS_GetMotionAccelNZ(ahrs) ((ahrs)->output.motion_accel_n[2])

#if AHRS_USE_MAGNETOMETER
// 磁力计（机体坐标系，纠偏后）[原始单位]
#define AHRS_GetMagX(ahrs)     ((ahrs)->output.mag_b[0])
#define AHRS_GetMagY(ahrs)     ((ahrs)->output.mag_b[1])
#define AHRS_GetMagZ(ahrs)     ((ahrs)->output.mag_b[2])
#endif

#endif // AHRS_H
