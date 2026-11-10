/**
 * @file    ahrs.h
 * @brief   AHRS 姿态解算（EKF 六轴：陀螺+加速度，无磁力计）
 * @note    从 leg_main 移植六轴实现；STM32F407 使用 math.h 软件欧拉角。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）
 */
#pragma once

#include "bmi088.h"
#include "kalman_filter.hpp"
#include "bsp_dwt.h"
#include "imu_temp.h"

#ifdef AHRS_USE_MAGNETOMETER
#undef AHRS_USE_MAGNETOMETER
#endif
#define AHRS_USE_MAGNETOMETER    0

#ifndef AHRS_ENABLE_VALIDATION
#define AHRS_ENABLE_VALIDATION    1
#endif

#ifndef AHRS_ENABLE_PROFILING
#define AHRS_ENABLE_PROFILING     1
#endif

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

#define AHRS_STATE_DIM     6
#define AHRS_OBS_DIM       3

#define AHRS_TEMP_CTRL_PERIOD_MS    750

/// DWT 任务完成时间，单位 us；包含抢占，不含 osDelay，首拍循环间隔为 0。
extern volatile float ahrs_task_elapsed_us;
extern volatile float ahrs_task_max_us;
extern volatile float ahrs_task_period_us;

#if AHRS_ENABLE_PROFILING
struct AHRSProfile {
    float imu_us;
    float imu_max_us;
    float ekf_us;
    float ekf_max_us;
    float euler_us;
    float euler_max_us;
};
extern volatile AHRSProfile ahrs_profile;
#endif

class AHRS {
public:
    using KF = KalmanFilter<AHRS_STATE_DIM, AHRS_OBS_DIM>;

    struct Output {
        float q[4];
        float euler[3];  ///< roll、pitch、yaw，单位 deg；yaw 位于 [-180, 180]。
        float yaw_total;
        float gyro_b[3];
        float accel_b[3];
        float motion_accel_b[3];
        float motion_accel_n[3];
    };

    struct Config {
        BMI088::AccRange accel_range;
        BMI088::GyroRange gyro_range;
        float board_yaw;
        float board_pitch;
        float board_roll;
        float accel_lpf_coef;
        float process_noise_quat;
        float process_noise_bias;
        float obs_noise_accel;
        uint32_t preheat_timeout_ms;
    };

    // —— 实例变量 ——
    Output output_ = {};
    uint32_t preheat_elapsed_ms_ = 0;

private:
    BMI088 bmi088_;
    KF kf_;
    IMUTemp imu_temp_;

    float install_rot_[9] = {};
    float accel_lpf_coef_ = 0.0f;
    float gyro_bias_[3] = {};
    uint32_t dwt_cnt_ = 0;
    int16_t yaw_round_count_ = 0;
    float yaw_last_ = 0.0f;

    float process_noise_quat_ = 0.0f;
    float process_noise_bias_ = 0.0f;
    float obs_noise_accel_ = 0.0f;
    uint32_t preheat_timeout_ms_ = 0;

#if AHRS_ENABLE_PROFILING
    float profile_cycles_to_us_ = 0.0f;
#endif

    Matrixf<AHRS_OBS_DIM, AHRS_OBS_DIM> r_accel_;

#if !AHRS_RTOS_SUPPORT
    uint32_t task_last_tick_ = 0;
#endif

    // —— static 函数 ——
    static void buildTransition(const Matrixf<AHRS_STATE_DIM, 1>& state, float* F, const float* u);
    /// H 为紧凑 3×4 四元数 Jacobian，两个零偏状态对应的恒零列不存储。
    static void observeGravity(const Matrixf<AHRS_STATE_DIM, 1>& state, float* H, float* h_x);
#if AHRS_RTOS_SUPPORT
    static void internalTask(void* arg);
#endif

    // —— 实例函数 ——
public:
    void init(const Config& config);
    void preheat();
    void calibrate();
    void start();
#if !AHRS_RTOS_SUPPORT
    void task();
#endif

private:
    void initQuaternion();
    void update();
};
