/**
 * @file    ahrs.cpp
 * @brief   AHRS 姿态解算实现（C → C++）
 */
#include "ahrs.h"
#include <string.h>
#include <math.h>

float CORDICMath::atan2(float y, float x) {
    return atan2f(y, x);
}

volatile float ahrs_task_elapsed_us = 0.0f;
volatile float ahrs_task_max_us = 0.0f;
volatile float ahrs_task_period_us = 0.0f;

#if AHRS_ENABLE_PROFILING
volatile AHRSProfile ahrs_profile = {};
#endif

namespace {

constexpr float DEG_TO_RAD = 0.017453292519943295f;
constexpr float RAD_TO_DEG = 57.29577951308232f;
constexpr uint32_t DEFAULT_PREHEAT_TIMEOUT_MS = 15000;
constexpr uint8_t PREHEAT_READY_COUNT = 3;
constexpr uint16_t GYRO_CALIBRATION_BATCH_SAMPLES = 750;
constexpr uint8_t GYRO_CALIBRATION_BATCH_COUNT = 8;
constexpr uint32_t GYRO_CALIBRATION_TIMEOUT_US = 12000000;
constexpr float GYRO_CALIBRATION_MAX_RANGE = 0.15f;

// 快速逆平方根（保持 C 原版位操作，memcpy 规避严格别名 UB）
float invSqrt(float x) {
    float halfx = 0.5f * x;
    int32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f375a86 - (i >> 1);
    memcpy(&x, &i, sizeof(x));
    x = x * (1.5f - halfx * x * x);
    return x;
}

// 四元数归一化
void quatNormalize(float* q) {
    float norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-6f) {
        q[0] *= norm; q[1] *= norm; q[2] *= norm; q[3] *= norm;
    }
}

// 四元数转欧拉角 (ZYX 顺序: Yaw-Pitch-Roll)
void quatToEuler(const float* q, float* roll, float* pitch, float* yaw,
                 CORDICMath* cordic) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    const float roll_y = 2.0f*(q0*q1 + q2*q3);
    const float roll_x = 1.0f - 2.0f*(q1*q1 + q2*q2);
    *roll = (cordic != nullptr ? cordic->atan2(roll_y, roll_x)
                              : atan2f(roll_y, roll_x)) * RAD_TO_DEG;

    float sinp = 2.0f*(q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }

    const float yaw_y = 2.0f*(q0*q3 + q1*q2);
    const float yaw_x = 1.0f - 2.0f*(q2*q2 + q3*q3);
    *yaw = (cordic != nullptr ? cordic->atan2(yaw_y, yaw_x)
                             : atan2f(yaw_y, yaw_x)) * RAD_TO_DEG;
}

// 3D 旋转矩阵乘法: out = R × v
void rotVec(const float* R, const float* v, float* out) {
    out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

}  // namespace

/*============================================
 * AHRS 六轴模型
 *
 * 状态向量（无磁力计）: x = [q0, q1, q2, q3, bx, by]^T  (n=6)
 ============================================*/

// EKF 预测: 计算 F 矩阵
void AHRS::buildTransition(const Matrixf<AHRS_STATE_DIM, 1>& state, float* F, const float* u) {
    /*
     * u[0] = dt；u[1] = wx；u[2] = wy；u[3] = wz
     */
    float dt = u[0];
    float wx = u[1];
    float wy = u[2];
    float wz = u[3];
    const int n = AHRS_STATE_DIM;

    memset(F, 0, n * n * sizeof(float));
    for (int i = 0; i < n; i++)
        F[i*n + i] = 1.0f;

    float half_dt = 0.5f * dt;

    F[0*n+1] = -half_dt * wx;
    F[0*n+2] = -half_dt * wy;
    F[0*n+3] = -half_dt * wz;

    F[1*n+0] =  half_dt * wx;
    F[1*n+2] =  half_dt * wz;
    F[1*n+3] = -half_dt * wy;

    F[2*n+0] =  half_dt * wy;
    F[2*n+1] = -half_dt * wz;
    F[2*n+3] =  half_dt * wx;

    F[3*n+0] =  half_dt * wz;
    F[3*n+1] =  half_dt * wy;
    F[3*n+2] = -half_dt * wx;

    float q0 = state[0][0], q1 = state[1][0], q2 = state[2][0], q3 = state[3][0];
    F[0*n+4] =  q1 * half_dt;   F[0*n+5] =  q2 * half_dt;
    F[1*n+4] = -q0 * half_dt;   F[1*n+5] =  q3 * half_dt;
    F[2*n+4] = -q3 * half_dt;   F[2*n+5] = -q0 * half_dt;
    F[3*n+4] =  q2 * half_dt;   F[3*n+5] = -q1 * half_dt;
}

// EKF 更新: 加速度计观测 Jacobian
void AHRS::observeGravity(const Matrixf<AHRS_STATE_DIM, 1>& state, float* H, float* h_x) {
    float q0 = state[0][0], q1 = state[1][0], q2 = state[2][0], q3 = state[3][0];
    const int n = 4;  // 仅存四元数列，零偏列恒为零。

    h_x[0] = 2.0f*(q1*q3 - q0*q2);
    h_x[1] = 2.0f*(q0*q1 + q2*q3);
    h_x[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    memset(H, 0, 3 * n * sizeof(float));

    H[0*n+0] = -2.0f*q2;  H[0*n+1] =  2.0f*q3;  H[0*n+2] = -2.0f*q0;  H[0*n+3] =  2.0f*q1;
    H[1*n+0] =  2.0f*q1;  H[1*n+1] =  2.0f*q0;  H[1*n+2] =  2.0f*q3;  H[1*n+3] =  2.0f*q2;
    H[2*n+0] =  2.0f*q0;  H[2*n+1] = -2.0f*q1;  H[2*n+2] = -2.0f*q2;  H[2*n+3] =  2.0f*q3;
}


/*============================================
 * 初始化
 ============================================*/

void AHRS::init(const Config& config) {
    cordic_ = config.cordic;
#if AHRS_ENABLE_PROFILING
    profile_cycles_to_us_ = 1000000.0f / static_cast<float>(HAL_RCC_GetSysClockFreq());
#endif
    BMI088::Config bmi088_config = {
        .spi_acc_config = {
            .hspi = &hspi1,
            .cs_port = GPIOA,
            .cs_pin = GPIO_PIN_4,   // C board ACC_CS
        },
        .spi_gyro_config = {
            .hspi = &hspi1,
            .cs_port = GPIOB,
            .cs_pin = GPIO_PIN_0,   // C board GYRO_CS
        },
        .accel_range = config.accel_range,
        .gyro_range = config.gyro_range,
    };
    bmi088_.init(bmi088_config);

    // 安装角旋转矩阵（顺序：先 roll(X) → 再 pitch(Y) → 再 yaw(Z)；Rz × Ry × Rx）
    float cy = cosf(config.board_yaw * DEG_TO_RAD);
    float sy = sinf(config.board_yaw * DEG_TO_RAD);
    float cp = cosf(config.board_pitch * DEG_TO_RAD);
    float sp = sinf(config.board_pitch * DEG_TO_RAD);
    float cr = cosf(config.board_roll * DEG_TO_RAD);
    float sr = sinf(config.board_roll * DEG_TO_RAD);

    install_rot_[0] = cy*cp;
    install_rot_[1] = -sy*cr + cy*sp*sr;
    install_rot_[2] = sy*sr + cy*sp*cr;
    install_rot_[3] = sy*cp;
    install_rot_[4] = cy*cr + sy*sp*sr;
    install_rot_[5] = -cy*sr + sy*sp*cr;
    install_rot_[6] = -sp;
    install_rot_[7] = cp*sr;
    install_rot_[8] = cp*cr;

    accel_lpf_coef_ = (config.accel_lpf_coef > 0.0f) ? config.accel_lpf_coef : 0.0085f;

    // EKF 噪声配置
    process_noise_quat_ = (config.process_noise_quat > 0.0f) ? config.process_noise_quat : 10.0f;
    process_noise_bias_ = (config.process_noise_bias > 0.0f) ? config.process_noise_bias : 0.001f;
    obs_noise_accel_ = (config.obs_noise_accel > 0.0f) ? config.obs_noise_accel : 1000000.0f;

    // 预热配置
    preheat_timeout_ms_ = (config.preheat_timeout_ms > 0)
        ? config.preheat_timeout_ms
        : DEFAULT_PREHEAT_TIMEOUT_MS;

    // R 矩阵（常量，只算一次）
    r_accel_ = Matrixf<AHRS_OBS_DIM, AHRS_OBS_DIM>::zeros();
    r_accel_[0][0] = obs_noise_accel_;
    r_accel_[1][1] = obs_noise_accel_;
    r_accel_[2][2] = obs_noise_accel_;

    // 卡尔曼滤波器
    const int n = AHRS_STATE_DIM;
    float x_init[AHRS_STATE_DIM];
    memset(x_init, 0, sizeof(x_init));
    x_init[0] = 1.0f;
    float P_init[AHRS_STATE_DIM * AHRS_STATE_DIM];
    memset(P_init, 0, sizeof(P_init));
    for (int i = 0; i < n; i++)
        P_init[i*n + i] = (i < 4) ? 100000.0f : 100.0f;

    KF::Config kf_config = {
        .x_init = x_init,
        .P_init = P_init,
        .lambda = 0.9996f,
    };
    kf_.init(kf_config);

    // 温度控制 (TIM10 CH1, PF6)
    extern TIM_HandleTypeDef htim10;
    IMUTemp::Config imu_temp_config = {
        .htim = &htim10,
        .channel = TIM_CHANNEL_1,
    };
    imu_temp_.init(imu_temp_config);

    // 对齐 C 版 AHRS_Register：注册完成后立即预热 + 陀螺仪校准 + 启动 RTOS 任务
    preheat();
    calibrate();
    start();
}

void AHRS::preheat() {
    uint32_t start = HAL_GetTick();
    uint8_t ready_count = 0;

    while (HAL_GetTick() - start < preheat_timeout_ms_) {
        bmi088_.readAll();
        imu_temp_.update(bmi088_.temperature);

        if (imu_temp_.isReady()) {
            if (++ready_count >= PREHEAT_READY_COUNT) break;
        } else {
            ready_count = 0;
        }

        HAL_Delay(AHRS_TEMP_CTRL_PERIOD_MS);
    }
    preheat_elapsed_ms_ = HAL_GetTick() - start; // 记录预热实际耗时 [ms]
    // 超时后仍继续校准，避免传感器异常导致启动永久卡住。
}

void AHRS::calibrate() {
    constexpr uint32_t sample_count =
        GYRO_CALIBRATION_BATCH_SAMPLES * GYRO_CALIBRATION_BATCH_COUNT;
    float gyro_sum[3] = {};
    float gyro_max[3] = {};
    float gyro_min[3] = {};
    uint32_t start_tick = DWT_GetTimeline_us();
    bool pass = false;

    bmi088_.setGyroOffset(0.0f, 0.0f, 0.0f);

    do {
        memset(gyro_sum, 0, sizeof(gyro_sum));

        for (uint8_t batch = 0; batch < GYRO_CALIBRATION_BATCH_COUNT; ++batch) {
            for (uint16_t i = 0; i < GYRO_CALIBRATION_BATCH_SAMPLES; ++i) {
                bmi088_.readGyro();

                if (batch == 0 && i == 0) {
                    gyro_max[0] = gyro_min[0] = bmi088_.gyro.x;
                    gyro_max[1] = gyro_min[1] = bmi088_.gyro.y;
                    gyro_max[2] = gyro_min[2] = bmi088_.gyro.z;
                } else {
                    if (bmi088_.gyro.x > gyro_max[0]) gyro_max[0] = bmi088_.gyro.x;
                    if (bmi088_.gyro.x < gyro_min[0]) gyro_min[0] = bmi088_.gyro.x;
                    if (bmi088_.gyro.y > gyro_max[1]) gyro_max[1] = bmi088_.gyro.y;
                    if (bmi088_.gyro.y < gyro_min[1]) gyro_min[1] = bmi088_.gyro.y;
                    if (bmi088_.gyro.z > gyro_max[2]) gyro_max[2] = bmi088_.gyro.z;
                    if (bmi088_.gyro.z < gyro_min[2]) gyro_min[2] = bmi088_.gyro.z;
                }

                gyro_sum[0] += bmi088_.gyro.x;
                gyro_sum[1] += bmi088_.gyro.y;
                gyro_sum[2] += bmi088_.gyro.z;
                DWT_Delay_ms(1);
            }

            bmi088_.readTemp();
            imu_temp_.update(bmi088_.temperature);
        }

        pass = (gyro_max[0] - gyro_min[0] < GYRO_CALIBRATION_MAX_RANGE) &&
               (gyro_max[1] - gyro_min[1] < GYRO_CALIBRATION_MAX_RANGE) &&
               (gyro_max[2] - gyro_min[2] < GYRO_CALIBRATION_MAX_RANGE);
    } while (!pass &&
             (DWT_GetTimeline_us() - start_tick) < GYRO_CALIBRATION_TIMEOUT_US);

    bmi088_.setGyroOffset(
        gyro_sum[0] / sample_count,
        gyro_sum[1] / sample_count,
        gyro_sum[2] / sample_count);

    initQuaternion();
    DWT_GetDeltaT(&dwt_cnt_);
}

void AHRS::start() {
#if AHRS_RTOS_SUPPORT
    osThreadAttr_t attr = {};
    attr.name = "AHRS";
    attr.stack_size = 4096;
    attr.priority = osPriorityAboveNormal;
    osThreadNew(internalTask, this, &attr);
#endif
}

#if !AHRS_RTOS_SUPPORT
void AHRS::task() {
    uint32_t tick = DWT_GetTimeline_us();
    if (tick - task_last_tick_ < 1000) return;  // 1ms 最小周期
    task_last_tick_ = tick;
    update();
}
#endif

/*============================================
 * 四元数初始化
 ============================================*/

void AHRS::initQuaternion() {
    float acc_sum[3] = {0};
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        bmi088_.readAll();
        acc_sum[0] += bmi088_.accel.x;
        acc_sum[1] += bmi088_.accel.y;
        acc_sum[2] += bmi088_.accel.z;
        DWT_Delay_ms(1);
    }

    float ax = acc_sum[0] / samples;
    float ay = acc_sum[1] / samples;
    float az = acc_sum[2] / samples;

    // 安装角旋转 (S → B)
    float accel_raw[3] = {ax, ay, az};
    float accel_rot[3];
    rotVec(install_rot_, accel_raw, accel_rot);
    ax = accel_rot[0];
    ay = accel_rot[1];
    az = accel_rot[2];

    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    float gravity[3] = {0, 0, 1};
    float axis[3];
    axis[0] = ay*gravity[2] - az*gravity[1];
    axis[1] = az*gravity[0] - ax*gravity[2];
    axis[2] = ax*gravity[1] - ay*gravity[0];

    float cos_angle = ax*gravity[0] + ay*gravity[1] + az*gravity[2];
    float angle = acosf(fabsf(cos_angle) > 0.9999f ? copysignf(0.9999f, cos_angle) : cos_angle);
    float half_sin = sinf(angle * 0.5f);
    float half_cos = cosf(angle * 0.5f);

    float axis_norm = invSqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);

    if (axis_norm > 1e-6f) {
        axis[0] *= axis_norm;
        axis[1] *= axis_norm;
        axis[2] *= axis_norm;
        kf_.x_[0][0] = half_cos;
        kf_.x_[1][0] = axis[0] * half_sin;
        kf_.x_[2][0] = axis[1] * half_sin;
        kf_.x_[3][0] = axis[2] * half_sin;
    } else {
        kf_.x_[0][0] = 1.0f;
        kf_.x_[1][0] = 0.0f;
        kf_.x_[2][0] = 0.0f;
        kf_.x_[3][0] = 0.0f;
    }

    float q[4] = {kf_.x_[0][0], kf_.x_[1][0], kf_.x_[2][0], kf_.x_[3][0]};
    quatNormalize(q);
    kf_.x_[0][0] = q[0];
    kf_.x_[1][0] = q[1];
    kf_.x_[2][0] = q[2];
    kf_.x_[3][0] = q[3];
}

/*============================================
 * 核心更新
 ============================================*/

void AHRS::update() {
    // 1. 计算时间步长 [s]
    float dt = DWT_GetDeltaT(&dwt_cnt_);
    const int n = AHRS_STATE_DIM;

    // 2. 读取传感器数据
#if AHRS_ENABLE_PROFILING
    const uint32_t imu_start = DWT->CYCCNT;
#endif
    bmi088_.readAll();
#if AHRS_ENABLE_PROFILING
    const uint32_t imu_cycles = DWT->CYCCNT - imu_start;
#endif
    float gx_raw = bmi088_.gyro.x;
    float gy_raw = bmi088_.gyro.y;
    float gz_raw = bmi088_.gyro.z;
    float ax_raw = bmi088_.accel.x;
    float ay_raw = bmi088_.accel.y;
    float az_raw = bmi088_.accel.z;

    // 3. 安装角旋转 (S → B)
    float gyro_raw[3] = {gx_raw, gy_raw, gz_raw};
    float accel_raw[3] = {ax_raw, ay_raw, az_raw};
    float gyro_rot[3], accel_rot[3];
    rotVec(install_rot_, gyro_raw, gyro_rot);
    rotVec(install_rot_, accel_raw, accel_rot);

    // 4. 直接扣除机体系残余零偏
    float gx = gyro_rot[0] - gyro_bias_[0];
    float gy = gyro_rot[1] - gyro_bias_[1];
    float gz = gyro_rot[2];

    // 存储纠偏后角速度 [rad/s]
    output_.gyro_b[0] = gx;
    output_.gyro_b[1] = gy;
    output_.gyro_b[2] = gz;

    // 5. 更新过程噪声 Q = Q_base * dt
#if AHRS_ENABLE_PROFILING
    const uint32_t ekf_start = DWT->CYCCNT;
#endif
    Matrixf<AHRS_STATE_DIM, AHRS_STATE_DIM> Q = Matrixf<AHRS_STATE_DIM, AHRS_STATE_DIM>::zeros();
    for (int i = 0; i < n; i++) {
        float noise = (i < 4) ? process_noise_quat_ : process_noise_bias_;
        Q[i][i] = noise * dt;
    }

    // 6. EKF 预测
    float u[4] = {dt, gx, gy, gz};
    Matrixf<AHRS_STATE_DIM, AHRS_STATE_DIM> F;
    buildTransition(kf_.x_, F[0], u);
    // 对比分支保留旧版 F*x 预测值；此次仅分离职责，不修正预测模型。
    const Matrixf<AHRS_STATE_DIM, 1> predicted_state = F * kf_.x_;
    kf_.predictEKF(predicted_state, F, Q);

    // 7. 加速度低通滤波
    float alpha = dt / (dt + accel_lpf_coef_);
    output_.accel_b[0] = output_.accel_b[0] * (1-alpha) + accel_rot[0] * alpha;
    output_.accel_b[1] = output_.accel_b[1] * (1-alpha) + accel_rot[1] * alpha;
    output_.accel_b[2] = output_.accel_b[2] * (1-alpha) + accel_rot[2] * alpha;

    // 8. 归一化加速度观测
    float norm_acc = invSqrt(output_.accel_b[0]*output_.accel_b[0]
                           + output_.accel_b[1]*output_.accel_b[1]
                           + output_.accel_b[2]*output_.accel_b[2]);
    Matrixf<AHRS_OBS_DIM, 1> z_accel;
    z_accel[0][0] = output_.accel_b[0] * norm_acc;
    z_accel[1][0] = output_.accel_b[1] * norm_acc;
    z_accel[2][0] = output_.accel_b[2] * norm_acc;

    // AHRS 在先验状态处生成重力观测值及 Jacobian。
    Matrixf<AHRS_OBS_DIM, 4> H;
    Matrixf<AHRS_OBS_DIM, 1> predicted_measurement;
    observeGravity(kf_.x_, H[0], predicted_measurement[0]);
    kf_.updateEKF(z_accel, predicted_measurement, H, r_accel_);

#if AHRS_ENABLE_VALIDATION
    // 协方差边界检查：防止 P 矩阵发散
    for (int i = 0; i < n; i++) {
        if (kf_.P_[i][i] > 100000.0f) {
            kf_.P_[i][i] = 100000.0f;
        }
    }
#endif

    // 12. 归一化四元数
    float q[4] = {kf_.x_[0][0], kf_.x_[1][0], kf_.x_[2][0], kf_.x_[3][0]};
    quatNormalize(q);
    kf_.x_[0][0] = q[0];
    kf_.x_[1][0] = q[1];
    kf_.x_[2][0] = q[2];
    kf_.x_[3][0] = q[3];

    // 13. 更新零偏估计
#if AHRS_ENABLE_PROFILING
    const uint32_t ekf_cycles = DWT->CYCCNT - ekf_start;
#endif
    gyro_bias_[0] = kf_.x_[4][0];
    gyro_bias_[1] = kf_.x_[5][0];

    // 14. 输出四元数
    memcpy(output_.q, q, 4 * sizeof(float));

    // 15. 四元数转欧拉角
#if AHRS_ENABLE_PROFILING
    const uint32_t euler_start = DWT->CYCCNT;
#endif
    quatToEuler(output_.q, &output_.euler[0], &output_.euler[1], &output_.euler[2],
                cordic_);
#if AHRS_ENABLE_PROFILING
    const uint32_t euler_cycles = DWT->CYCCNT - euler_start;
    const float imu_us = static_cast<float>(imu_cycles) * profile_cycles_to_us_;
    const float ekf_us = static_cast<float>(ekf_cycles) * profile_cycles_to_us_;
    const float euler_us = static_cast<float>(euler_cycles) * profile_cycles_to_us_;
    ahrs_profile.imu_us = imu_us;
    ahrs_profile.ekf_us = ekf_us;
    ahrs_profile.euler_us = euler_us;
    if (imu_us > ahrs_profile.imu_max_us) ahrs_profile.imu_max_us = imu_us;
    if (ekf_us > ahrs_profile.ekf_max_us) ahrs_profile.ekf_max_us = ekf_us;
    if (euler_us > ahrs_profile.euler_max_us) ahrs_profile.euler_max_us = euler_us;
#endif

    // 16. 累计偏航角 (处理角度跳变)
    float yaw = output_.euler[2];
    float diff = yaw - yaw_last_;
    if (diff > 180.0f) yaw_round_count_--;
    else if (diff < -180.0f) yaw_round_count_++;
    output_.yaw_total = 360.0f * yaw_round_count_ + yaw;
    yaw_last_ = yaw;

    // 17. 计算运动加速度 [m/s²] (去除重力)
    float q0 = output_.q[0], q1 = output_.q[1];
    float q2 = output_.q[2], q3 = output_.q[3];
    float gravity_b[3] = {
        2.0f*(q1*q3 - q0*q2) * 9.81f,
        2.0f*(q0*q1 + q2*q3) * 9.81f,
        (q0*q0 - q1*q1 - q2*q2 + q3*q3) * 9.81f
    };
    output_.motion_accel_b[0] = accel_rot[0] - gravity_b[0];
    output_.motion_accel_b[1] = accel_rot[1] - gravity_b[1];
    output_.motion_accel_b[2] = accel_rot[2] - gravity_b[2];

    // 18. 转换到世界坐标系
    float R_mat[9] = {
        1.0f - 2.0f*(q2*q2 + q3*q3),  2.0f*(q1*q2 - q0*q3),        2.0f*(q1*q3 + q0*q2),
        2.0f*(q1*q2 + q0*q3),         1.0f - 2.0f*(q1*q1 + q3*q3), 2.0f*(q2*q3 - q0*q1),
        2.0f*(q1*q3 - q0*q2),         2.0f*(q2*q3 + q0*q1),        1.0f - 2.0f*(q1*q1 + q2*q2)
    };
    rotVec(R_mat, output_.motion_accel_b, output_.motion_accel_n);
}

/*============================================
 * RTOS 任务
 ============================================*/

#if AHRS_RTOS_SUPPORT
void AHRS::internalTask(void* arg) {
    AHRS* ahrs = static_cast<AHRS*>(arg);
    uint32_t last_temp_tick = 0;
    uint32_t previous_start = 0;
    uint8_t timing_started = 0;
    for (;;) {
        const float period_us = DWT_GetDeltaT(&previous_start) * 1000000.0f;
        uint32_t timing_start = previous_start;
        ahrs_task_period_us = timing_started ? period_us : 0.0f;
        timing_started = 1;
        ahrs->update();
        if (HAL_GetTick() - last_temp_tick >= AHRS_TEMP_CTRL_PERIOD_MS) {
            last_temp_tick = HAL_GetTick();
            ahrs->imu_temp_.update(ahrs->bmi088_.temperature);
        }
        const float elapsed_us = DWT_GetDeltaT(&timing_start) * 1000000.0f;
        ahrs_task_elapsed_us = elapsed_us;
        if (elapsed_us > ahrs_task_max_us)
            ahrs_task_max_us = elapsed_us;
        osDelay(1);
    }
}
#endif
