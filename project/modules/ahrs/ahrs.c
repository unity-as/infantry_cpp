#include "ahrs.h"
#include "imu_temp.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define DEG_TO_RAD  0.017453292519943295f
#define RAD_TO_DEG  57.29577951308232f

/*============================================
 * 内部全局变量
 ============================================*/

static AHRS_Instance *ahrs_global;

/*============================================
 * 内部数学函数
 ============================================*/

// 快速逆平方根
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    int32_t i = *(int32_t *)&x;
    i = 0x5f375a86 - (i >> 1);
    x = *(float *)&i;
    x = x * (1.5f - halfx * x * x);
    return x;
}

// 四元数归一化
static void quat_normalize(float *q)
{
    float norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-6f) {
        q[0] *= norm; q[1] *= norm; q[2] *= norm; q[3] *= norm;
    }
}

// 四元数转欧拉角 (ZYX 顺序: Yaw-Pitch-Roll)
static void quat_to_euler(const float *q, float *roll, float *pitch, float *yaw)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // Roll (绕 X 轴)
    *roll = atan2f(2.0f*(q0*q1 + q2*q3),
                   1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;

    // Pitch (绕 Y 轴) - 处理万向锁
    float sinp = 2.0f*(q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }

    // Yaw (绕 Z 轴)
    *yaw = atan2f(2.0f*(q0*q3 + q1*q2),
                  1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}

// 3D 旋转矩阵乘法: out = R × v
static void rot_vec(const float *R, const float *v, float *out)
{
    out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

/*============================================
 * EKF 回调函数
 *
 * AHRS 状态向量:
 *   无磁力计: x = [q0, q1, q2, q3, bx, by]^T  (n=6)
 *   有磁力计: x = [q0, q1, q2, q3, bx, by, bz]^T  (n=7)
 ============================================*/

/*---------- EKF 预测: 计算 F 矩阵 ----------*/

static void AHRS_Predict_Callback(KF_Instance *kf, float *F, const float *u)
{
    /*
     * u[0] = dt (时间步长 [s])
     * u[1] = wx (陀螺仪 x [rad/s])
     * u[2] = wy (陀螺仪 y [rad/s])
     * u[3] = wz (陀螺仪 z [rad/s])
     */
    float dt = u[0];
    float wx = u[1];
    float wy = u[2];
    float wz = u[3];
    int n = kf->n;

    // 初始化 F 为单位矩阵
    memset(F, 0, n * n * sizeof(float));
    for (int i = 0; i < n; i++)
        F[i*n + i] = 1.0f;

    // 四元数状态转移 Jacobian
    // 离散化: q_new = q + 0.5*Ω*q*dt
    // 其中 Ω 是四元数乘法矩阵

    float half_dt = 0.5f * dt;

    // F[0*4+1..3] = -0.5*dt*[wx, wy, wz]
    F[0*n+1] = -half_dt * wx;
    F[0*n+2] = -half_dt * wy;
    F[0*n+3] = -half_dt * wz;

    // F[1*4+0,2,3]
    F[1*n+0] =  half_dt * wx;
    F[1*n+2] =  half_dt * wz;
    F[1*n+3] = -half_dt * wy;

    // F[2*4+0,1,3]
    F[2*n+0] =  half_dt * wy;
    F[2*n+1] = -half_dt * wz;
    F[2*n+3] =  half_dt * wx;

    // F[3*4+0,1,2]
    F[3*n+0] =  half_dt * wz;
    F[3*n+1] =  half_dt * wy;
    F[3*n+2] = -half_dt * wx;

#if AHRS_USE_MAGNETOMETER
    // 有磁力计: 零偏维度为 3 (bx, by, bz)
    float q0 = kf->x[0], q1 = kf->x[1], q2 = kf->x[2], q3 = kf->x[3];
    // 偏置耦合: F[0:4][4:7] = -0.5*dt * ∂(Ωq)/∂ω (对齐旧代码)
    F[0*n+4] =  q1 * half_dt;   F[0*n+5] =  q2 * half_dt;   F[0*n+6] =  q3 * half_dt;
    F[1*n+4] = -q0 * half_dt;   F[1*n+5] =  q3 * half_dt;   F[1*n+6] = -q2 * half_dt;
    F[2*n+4] = -q3 * half_dt;   F[2*n+5] = -q0 * half_dt;   F[2*n+6] =  q1 * half_dt;
    F[3*n+4] =  q2 * half_dt;   F[3*n+5] = -q1 * half_dt;   F[3*n+6] = -q0 * half_dt;
#else
    // 无磁力计: 零偏维度为 2 (bx, by)，偏航零偏不可观测
    float q0 = kf->x[0], q1 = kf->x[1], q2 = kf->x[2], q3 = kf->x[3];
    // 偏置耦合: F[0:4][4:6] = -0.5*dt * ∂(Ωq)/∂ω (对齐旧代码)
    F[0*n+4] =  q1 * half_dt;   F[0*n+5] =  q2 * half_dt;
    F[1*n+4] = -q0 * half_dt;   F[1*n+5] =  q3 * half_dt;
    F[2*n+4] = -q3 * half_dt;   F[2*n+5] = -q0 * half_dt;
    F[3*n+4] =  q2 * half_dt;   F[3*n+5] = -q1 * half_dt;
#endif
}

/*---------- EKF 更新: 加速度计观测 Jacobian ----------*/

static void AHRS_Update_Accel_Callback(KF_Instance *kf, float *H, float *h_x, const float *z)
{
    /*
     * 观测: 归一化加速度 (世界坐标系重力方向)
     * 预测观测: h(x) = R(q)^T × [0, 0, 1]
     *
     * h_x[0..2] = 预测的归一化加速度
     * H[0..2*n] = ∂h/∂x
     */
    (void)z;  // z 在此回调中不使用
    float q0 = kf->x[0], q1 = kf->x[1], q2 = kf->x[2], q3 = kf->x[3];
    int n = kf->n;

    // h(x) 预测观测值
    h_x[0] = 2.0f*(q1*q3 - q0*q2);
    h_x[1] = 2.0f*(q0*q1 + q2*q3);
    h_x[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // H 矩阵 (3×n)
    memset(H, 0, 3 * n * sizeof(float));

    // ∂h0/∂q
    H[0*n+0] = -2.0f*q2;  H[0*n+1] =  2.0f*q3;  H[0*n+2] = -2.0f*q0;  H[0*n+3] =  2.0f*q1;

    // ∂h1/∂q
    H[1*n+0] =  2.0f*q1;  H[1*n+1] =  2.0f*q0;  H[1*n+2] =  2.0f*q3;  H[1*n+3] =  2.0f*q2;

    // ∂h2/∂q
    H[2*n+0] =  2.0f*q0;  H[2*n+1] = -2.0f*q1;  H[2*n+2] = -2.0f*q2;  H[2*n+3] =  2.0f*q3;
}

#if AHRS_USE_MAGNETOMETER
/*---------- EKF 更新: 磁力计观测 Jacobian ----------*/

static void AHRS_Update_Mag_Callback(KF_Instance *kf, float *H, float *h_x, const float *z)
{
    /*
     * 观测: 归一化磁场 (假设地磁场在 X 方向)
     * 预测观测: h(x) = R(q)^T × [1, 0, 0]
     */
    (void)z;
    float q0 = kf->x[0], q1 = kf->x[1], q2 = kf->x[2], q3 = kf->x[3];
    int n = kf->n;

    // h(x) 预测观测值
    h_x[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    h_x[1] = 2.0f*(q1*q2 + q0*q3);
    h_x[2] = 2.0f*(q1*q3 - q0*q2);

    // H 矩阵 (3×n)
    memset(H, 0, 3 * n * sizeof(float));

    // ∂h0/∂q
    H[0*n+0] =  2.0f*q0;  H[0*n+1] =  2.0f*q1;
    H[0*n+2] = -2.0f*q2;  H[0*n+3] = -2.0f*q3;

    // ∂h1/∂q
    H[1*n+0] =  2.0f*q3;  H[1*n+1] =  2.0f*q2;
    H[1*n+2] =  2.0f*q1;  H[1*n+3] =  2.0f*q0;

    // ∂h2/∂q
    H[2*n+0] = -2.0f*q2;  H[2*n+1] =  2.0f*q3;
    H[2*n+2] = -2.0f*q0;  H[2*n+3] =  2.0f*q1;
}

/*---------- EKF 更新: 6 维联合观测 (加速度计+磁力计) ----------*/

static void AHRS_Update_6D_Callback(KF_Instance *kf, float *H, float *h_x, const float *z)
{
    /*
     * 观测: z = [ax, ay, az, mx, my, mz]
     * 预测观测: h(x) = [R(q)^T×[0,0,1], R(q)^T×[1,0,0]]
     * H 矩阵: 6×n
     */
    (void)z;
    float q0 = kf->x[0], q1 = kf->x[1], q2 = kf->x[2], q3 = kf->x[3];
    int n = kf->n;

    // h(x) 前 3 维: 加速度计预测
    h_x[0] = 2.0f*(q1*q3 - q0*q2);
    h_x[1] = 2.0f*(q0*q1 + q2*q3);
    h_x[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // h(x) 后 3 维: 磁力计预测
    h_x[3] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    h_x[4] = 2.0f*(q1*q2 + q0*q3);
    h_x[5] = 2.0f*(q1*q3 - q0*q2);

    // H 矩阵 (6×n)
    memset(H, 0, 6 * n * sizeof(float));

    // 加速度计 Jacobian (行 0-2)
    H[0*n+0] = -2.0f*q2;  H[0*n+1] =  2.0f*q3;  H[0*n+2] = -2.0f*q0;  H[0*n+3] =  2.0f*q1;
    H[1*n+0] =  2.0f*q1;  H[1*n+1] =  2.0f*q0;  H[1*n+2] =  2.0f*q3;  H[1*n+3] =  2.0f*q2;
    H[2*n+0] =  2.0f*q0;  H[2*n+1] = -2.0f*q1;  H[2*n+2] = -2.0f*q2;  H[2*n+3] =  2.0f*q3;

    // 磁力计 Jacobian (行 3-5)
    H[3*n+0] =  2.0f*q0;  H[3*n+1] =  2.0f*q1;  H[3*n+2] = -2.0f*q2;  H[3*n+3] = -2.0f*q3;
    H[4*n+0] =  2.0f*q3;  H[4*n+1] =  2.0f*q2;  H[4*n+2] =  2.0f*q1;  H[4*n+3] =  2.0f*q0;
    H[5*n+0] = -2.0f*q2;  H[5*n+1] =  2.0f*q3;  H[5*n+2] = -2.0f*q0;  H[5*n+3] =  2.0f*q1;
}
#endif

/*============================================
 * 初始化
 ============================================*/

// 使用加速度计初始化四元数
static void AHRS_Init_Quaternion(AHRS_Instance *ahrs)
{
    float acc_sum[3] = {0};
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        BMI088_Read_All(ahrs->bmi088);
        acc_sum[0] += ahrs->bmi088->accel.x;
        acc_sum[1] += ahrs->bmi088->accel.y;
        acc_sum[2] += ahrs->bmi088->accel.z;
        DWT_Delay_ms(1);
    }

    // 计算平均值
    float ax = acc_sum[0] / samples;
    float ay = acc_sum[1] / samples;
    float az = acc_sum[2] / samples;

    // 安装角旋转 (S → B)
    float accel_raw[3] = {ax, ay, az};
    float accel_rot[3];
    rot_vec(ahrs->install_rot, accel_raw, accel_rot);
    ax = accel_rot[0];
    ay = accel_rot[1];
    az = accel_rot[2];

    // 归一化
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    // 计算从当前加速度方向到重力方向的旋转
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
        ahrs->kf->x[0] = half_cos;
        ahrs->kf->x[1] = axis[0] * half_sin;
        ahrs->kf->x[2] = axis[1] * half_sin;
        ahrs->kf->x[3] = axis[2] * half_sin;
    } else {
        ahrs->kf->x[0] = 1.0f;
        ahrs->kf->x[1] = 0.0f;
        ahrs->kf->x[2] = 0.0f;
        ahrs->kf->x[3] = 0.0f;
    }

    quat_normalize(ahrs->kf->x);
}

/*============================================
 * 核心更新
 ============================================*/

static void AHRS_Update(AHRS_Instance *ahrs)
{
    if (!ahrs) return;

    // 1. 计算时间步长 [s]
    float dt = DWT_GetDeltaT(&ahrs->dwt_cnt);
    int n = ahrs->kf->n;
    int m = ahrs->kf->m;

    // 2. 读取传感器数据
    BMI088_Read_All(ahrs->bmi088);
    float gx_raw = ahrs->bmi088->gyro.x;
    float gy_raw = ahrs->bmi088->gyro.y;
    float gz_raw = ahrs->bmi088->gyro.z;
    float ax_raw = ahrs->bmi088->accel.x;
    float ay_raw = ahrs->bmi088->accel.y;
    float az_raw = ahrs->bmi088->accel.z;

    // 3. 安装角旋转 (S → B: 传感器坐标系 → 机体坐标系)
    float gyro_raw[3] = {gx_raw, gy_raw, gz_raw};
    float accel_raw[3] = {ax_raw, ay_raw, az_raw};
    float gyro_rot[3], accel_rot[3];
    rot_vec(ahrs->install_rot, gyro_raw, gyro_rot);
    rot_vec(ahrs->install_rot, accel_raw, accel_rot);

    // 4. 减去零偏（零偏也要旋转到 body frame）
    float bias_rot[3];
    rot_vec(ahrs->install_rot, ahrs->gyro_bias, bias_rot);
    float gx = gyro_rot[0] - bias_rot[0];
    float gy = gyro_rot[1] - bias_rot[1];
    float gz = gyro_rot[2];
#if AHRS_USE_MAGNETOMETER
    gz -= bias_rot[2];
#endif

    // 存储纠偏后角速度 [rad/s]
    ahrs->output.gyro_b[0] = gx;
    ahrs->output.gyro_b[1] = gy;
    ahrs->output.gyro_b[2] = gz;

    // 5. 更新过程噪声 Q = Q_base * dt
    memset(ahrs->Q_data, 0, n * n * sizeof(float));
    for (int i = 0; i < n; i++) {
        float noise = (i < 4) ? ahrs->process_noise_quat : ahrs->process_noise_bias;
        ahrs->Q_data[i*n + i] = noise * dt;
    }

    // 6. EKF 预测
    float u[4] = {dt, gx, gy, gz};
    KF_Predict_EKF(ahrs->kf, ahrs->Q_data, u);

    // 7. 加速度低通滤波
    float alpha = dt / (dt + ahrs->accel_lpf_coef);
    ahrs->output.accel_b[0] = ahrs->output.accel_b[0] * (1-alpha) + accel_rot[0] * alpha;
    ahrs->output.accel_b[1] = ahrs->output.accel_b[1] * (1-alpha) + accel_rot[1] * alpha;
    ahrs->output.accel_b[2] = ahrs->output.accel_b[2] * (1-alpha) + accel_rot[2] * alpha;

    // 8. 归一化加速度观测
    float norm_acc = invSqrt(ahrs->output.accel_b[0]*ahrs->output.accel_b[0]
                           + ahrs->output.accel_b[1]*ahrs->output.accel_b[1]
                           + ahrs->output.accel_b[2]*ahrs->output.accel_b[2]);
    float z_accel[3];
    z_accel[0] = ahrs->output.accel_b[0] * norm_acc;
    z_accel[1] = ahrs->output.accel_b[1] * norm_acc;
    z_accel[2] = ahrs->output.accel_b[2] * norm_acc;

#if AHRS_USE_MAGNETOMETER
    // 9. 读取磁力计数据
    IST8310_Data_t mag_data;
    IST8310_Acquire(ahrs->ist8310, &mag_data);

    // 安装角旋转 (S → B)
    float mag_raw[3] = {mag_data.mag[0], mag_data.mag[1], mag_data.mag[2]};
    float mag_rot[3];
    rot_vec(ahrs->install_rot, mag_raw, mag_rot);

    // 存储纠偏后磁力计 [原始单位]
    ahrs->output.mag_b[0] = mag_rot[0];
    ahrs->output.mag_b[1] = mag_rot[1];
    ahrs->output.mag_b[2] = mag_rot[2];

    // 10. 归一化磁力计观测
    float norm_mag = invSqrt(mag_rot[0]*mag_rot[0] + mag_rot[1]*mag_rot[1] + mag_rot[2]*mag_rot[2]);
    float z_mag[3];
    z_mag[0] = mag_rot[0] * norm_mag;
    z_mag[1] = mag_rot[1] * norm_mag;
    z_mag[2] = mag_rot[2] * norm_mag;

    // 11. 构建 6 维联合观测向量
    float z_6d[6] = {z_accel[0], z_accel[1], z_accel[2], z_mag[0], z_mag[1], z_mag[2]};

    // 12. 构建 6×6 R 矩阵
    float R_6d[36] = {0};
    R_6d[0*6+0] = ahrs->obs_noise_accel;   // ax
    R_6d[1*6+1] = ahrs->obs_noise_accel;   // ay
    R_6d[2*6+2] = ahrs->obs_noise_accel;   // az
    R_6d[3*6+3] = ahrs->obs_noise_mag;     // mx
    R_6d[4*6+4] = ahrs->obs_noise_mag;     // my
    R_6d[5*6+5] = ahrs->obs_noise_mag;     // mz

    // 13. EKF 更新 - 6 维联合观测
    KF_Set_Update_Callback(ahrs->kf, AHRS_Update_6D_Callback);
    KF_Update_EKF(ahrs->kf, z_6d, R_6d, NULL);
#else
    // 仅加速度计观测
    KF_Set_Update_Callback(ahrs->kf, AHRS_Update_Accel_Callback);
    KF_Update_EKF(ahrs->kf, z_accel, ahrs->R_accel, NULL);
#endif

#if AHRS_ENABLE_VALIDATION
    // 协方差边界检查：防止 P 矩阵发散
    for (int i = 0; i < n; i++) {
        if (ahrs->kf->P[i*n + i] > 100000.0f) {
            ahrs->kf->P[i*n + i] = 100000.0f;
        }
    }
#endif

    // 12. 归一化四元数
    quat_normalize(ahrs->kf->x);

    // 13. 更新零偏估计
    ahrs->gyro_bias[0] = ahrs->kf->x[4];
    ahrs->gyro_bias[1] = ahrs->kf->x[5];
#if AHRS_USE_MAGNETOMETER
    ahrs->gyro_bias[2] = ahrs->kf->x[6];
#endif

    // 14. 输出四元数
    memcpy(ahrs->output.q, ahrs->kf->x, 4 * sizeof(float));

    // 15. 四元数转欧拉角
    quat_to_euler(ahrs->output.q,
                  &ahrs->output.euler[0],
                  &ahrs->output.euler[1],
                  &ahrs->output.euler[2]);

    // 16. 累计偏航角 (处理角度跳变)
    float yaw = ahrs->output.euler[2];
    float diff = yaw - ahrs->yaw_last;
    if (diff > 180.0f) ahrs->yaw_round_count--;
    else if (diff < -180.0f) ahrs->yaw_round_count++;
    ahrs->output.yaw_total = 360.0f * ahrs->yaw_round_count + yaw;
    ahrs->yaw_last = yaw;

    // 17. 计算运动加速度 [m/s²] (去除重力)
    float q0 = ahrs->output.q[0], q1 = ahrs->output.q[1];
    float q2 = ahrs->output.q[2], q3 = ahrs->output.q[3];
    float gravity_b[3] = {
        2.0f*(q1*q3 - q0*q2) * 9.81f,
        2.0f*(q0*q1 + q2*q3) * 9.81f,
        (q0*q0 - q1*q1 - q2*q2 + q3*q3) * 9.81f
    };
    ahrs->output.motion_accel_b[0] = accel_rot[0] - gravity_b[0];
    ahrs->output.motion_accel_b[1] = accel_rot[1] - gravity_b[1];
    ahrs->output.motion_accel_b[2] = accel_rot[2] - gravity_b[2];

    // 18. 转换到世界坐标系
    float R_mat[9] = {
        1.0f - 2.0f*(q2*q2 + q3*q3),  2.0f*(q1*q2 - q0*q3),        2.0f*(q1*q3 + q0*q2),
        2.0f*(q1*q2 + q0*q3),         1.0f - 2.0f*(q1*q1 + q3*q3), 2.0f*(q2*q3 - q0*q1),
        2.0f*(q1*q3 - q0*q2),         2.0f*(q2*q3 + q0*q1),        1.0f - 2.0f*(q1*q1 + q2*q2)
    };
    rot_vec(R_mat, ahrs->output.motion_accel_b, ahrs->output.motion_accel_n);
}

/*============================================
 * 注册与任务
 ============================================*/

#if AHRS_RTOS_SUPPORT
static void AHRS_InternalTask(void *arg)
{
    AHRS_Instance *ahrs = (AHRS_Instance *)arg;
    uint32_t last_temp_tick = 0;
    for (;;) {
        AHRS_Update(ahrs);
        // 温控: AHRS_TEMP_CTRL_PERIOD_MS 周期
        if (ahrs->imu_temp && (HAL_GetTick() - last_temp_tick >= AHRS_TEMP_CTRL_PERIOD_MS)) {
            last_temp_tick = HAL_GetTick();
            IMU_Temp_Update((IMU_Temp_Instance *)ahrs->imu_temp, ahrs->bmi088->temperature);
        }
        osDelay(1);
    }
}
#endif

AHRS_Instance *AHRS_Register(AHRS_Init_Config_s *config)
{
    AHRS_Instance *ahrs = (AHRS_Instance *)malloc(sizeof(AHRS_Instance));
    if (!ahrs) return NULL;
    memset(ahrs, 0, sizeof(AHRS_Instance));

    BMI088_Init_Config_s bmi088_config = {
        .spi_acc_config = {
            .hspi = &hspi1,
            .cs_port = GPIOA,
            .cs_pin = GPIO_PIN_4,
        },
        .spi_gyro_config = {
            .hspi = &hspi1,
            .cs_port = GPIOB,
            .cs_pin = GPIO_PIN_0,
        },
        .accel_range = config->accel_range,
        .gyro_range = config->gyro_range,
    };
    ahrs->bmi088 = BMI088_Register(&bmi088_config);
    if (!ahrs->bmi088) { free(ahrs); return NULL; }

#if AHRS_USE_MAGNETOMETER
    IST8310_Init_Config_s ist8310_config = {
        .i2c_config = {
            .hi2c = &hi2c3,
            .dev_addr = IST8310_I2C_ADDR << 1,
        },
        .rst_config = {
            .GPIOx = GPIOG,
            .GPIO_Pin = GPIO_PIN_6,
        },
    };
    ahrs->ist8310 = IST8310_Register(&ist8310_config);
    if (!ahrs->ist8310) { free(ahrs); return NULL; }
#endif

    // 安装角旋转矩阵
    // 顺序：先 roll(X) → 再 pitch(Y) → 再 yaw(Z)
    // 矩阵从右到左相乘：Rz × Ry × Rx
    float cy = cosf(config->board_yaw * DEG_TO_RAD);
    float sy = sinf(config->board_yaw * DEG_TO_RAD);
    float cp = cosf(config->board_pitch * DEG_TO_RAD);
    float sp = sinf(config->board_pitch * DEG_TO_RAD);
    float cr = cosf(config->board_roll * DEG_TO_RAD);
    float sr = sinf(config->board_roll * DEG_TO_RAD);

    ahrs->install_rot[0] = cy*cp;
    ahrs->install_rot[1] = -sy*cr + cy*sp*sr;
    ahrs->install_rot[2] = sy*sr + cy*sp*cr;
    ahrs->install_rot[3] = sy*cp;
    ahrs->install_rot[4] = cy*cr + sy*sp*sr;
    ahrs->install_rot[5] = -cy*sr + sy*sp*cr;
    ahrs->install_rot[6] = -sp;
    ahrs->install_rot[7] = cp*sr;
    ahrs->install_rot[8] = cp*cr;

    ahrs->accel_lpf_coef = (config->accel_lpf_coef > 0.0f) ? config->accel_lpf_coef : 0.0085f;

    // EKF 噪声配置
    ahrs->process_noise_quat = (config->process_noise_quat > 0.0f) ? config->process_noise_quat : 10.0f;
    ahrs->process_noise_bias = (config->process_noise_bias > 0.0f) ? config->process_noise_bias : 0.001f;
    ahrs->obs_noise_accel = (config->obs_noise_accel > 0.0f) ? config->obs_noise_accel : 1000000.0f;
#if AHRS_USE_MAGNETOMETER
    ahrs->obs_noise_mag = (config->obs_noise_mag > 0.0f) ? config->obs_noise_mag : 1000000.0f;
#endif

    // 预热配置
    ahrs->preheat_timeout_ms = (config->preheat_timeout_ms > 0) ? config->preheat_timeout_ms : 30000;

    // 构建 R 矩阵 (常量，只算一次)
    memset(ahrs->R_accel, 0, sizeof(ahrs->R_accel));
    ahrs->R_accel[0] = ahrs->obs_noise_accel;
    ahrs->R_accel[4] = ahrs->obs_noise_accel;
    ahrs->R_accel[8] = ahrs->obs_noise_accel;

    // 卡尔曼滤波器
    int n = AHRS_STATE_DIM;
    int m = AHRS_OBS_DIM;
    float x_init[AHRS_STATE_DIM] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float P_init[AHRS_STATE_DIM * AHRS_STATE_DIM];
    memset(P_init, 0, sizeof(P_init));
    for (int i = 0; i < n; i++)
        P_init[i*n + i] = (i < 4) ? 100000.0f : 100.0f;

    KF_Init_Config_s kf_config = {
        .n = n, .m = m,
        .x_init = x_init, .P_init = P_init,
        .lambda = 0.9996f,
    };
    ahrs->kf = KF_Register(&kf_config);
    if (!ahrs->kf) { free(ahrs); return NULL; }
    KF_Set_Predict_Callback(ahrs->kf, AHRS_Predict_Callback);

    // 温度控制 (TIM10 CH1, PF6)
    extern TIM_HandleTypeDef htim10;
    ahrs->imu_temp = IMU_Temp_Init(&htim10, TIM_CHANNEL_1);

    ahrs_global = ahrs;

    AHRS_Preheat(ahrs);    // 阻塞: 加热到 40°C
    AHRS_Calibrate(ahrs);  // 阻塞: 陀螺仪校准
    AHRS_Start(ahrs);      // 创建 RTOS 任务

    return ahrs;
}

void AHRS_Preheat(AHRS_Instance *ahrs)
{
    if (!ahrs || !ahrs->bmi088 || !ahrs->imu_temp) return;

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < ahrs->preheat_timeout_ms) {
        BMI088_Read_All(ahrs->bmi088);
        IMU_Temp_Update((IMU_Temp_Instance *)ahrs->imu_temp, ahrs->bmi088->temperature);
        if (IMU_Temp_IsReady((IMU_Temp_Instance *)ahrs->imu_temp)) break;
        HAL_Delay(AHRS_TEMP_CTRL_PERIOD_MS);
    }
    // 超时也继续，不阻塞启动
}

void AHRS_Calibrate(AHRS_Instance *ahrs)
{
    if (!ahrs || !ahrs->bmi088) return;
    BMI088_Calibrate(ahrs->bmi088);
    AHRS_Init_Quaternion(ahrs);
    DWT_GetDeltaT(&ahrs->dwt_cnt);
}

void AHRS_Start(AHRS_Instance *ahrs)
{
    if (!ahrs) return;
#if AHRS_RTOS_SUPPORT
    const osThreadAttr_t attr = {
        .name = "AHRS",
        .stack_size = 4096,
        .priority = osPriorityAboveNormal,
    };
    osThreadNew(AHRS_InternalTask, ahrs, &attr);
#endif
}

void AHRS_Free(AHRS_Instance *ahrs)
{
    if (!ahrs) return;
    KF_Free(ahrs->kf);
    if (ahrs->imu_temp) free(ahrs->imu_temp);
    free(ahrs);
}

/*---------- 裸机 Task ----------*/

#if !AHRS_RTOS_SUPPORT
void AHRS_Task(void)
{
    static uint32_t last_tick = 0;
    uint32_t tick = DWT_GetTimeline_us();
    if (tick - last_tick < 1000) return;  // 1ms 最小周期
    last_tick = tick;
    AHRS_Update(ahrs_global);
}
#endif