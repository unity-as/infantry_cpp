#include "kalman_filter.h"
#include <stdlib.h>
#include <string.h>
#include "arm_math.h"

#define MAT_MAX_N 16

/*============================================
 * 矩阵辅助函数
 * 注意: arm_mat_inverse_f32 是原地操作
 *   src 矩阵被分解，结果存回 src
 *   输出矩阵初始为单位阵，求逆后变为 A^(-1)
 ============================================*/

static void mat_zero(float *A, int n)
{
    memset(A, 0, n * n * sizeof(float));
}

static void mat_eye(float *A, int n)
{
    mat_zero(A, n);
    for (int i = 0; i < n; i++)
        A[i * n + i] = 1.0f;
}

static void mat_copy(float *dst, const float *src, int n)
{
    memcpy(dst, src, n * sizeof(float));
}

static void mat_copy_nn(float *dst, const float *src, int n)
{
    memcpy(dst, src, n * n * sizeof(float));
}

// CMSIS DSP 包装: 矩阵乘法 C = A × B
// A: m×n, B: n×p, C: m×p
#define MAT_MUL(C, A, B, m, n, p) do { \
    arm_matrix_instance_f32 _a, _b, _c; \
    arm_mat_init_f32(&_a, m, n, (float *)(A)); \
    arm_mat_init_f32(&_b, n, p, (float *)(B)); \
    arm_mat_init_f32(&_c, m, p, C); \
    arm_mat_mult_f32(&_a, &_b, &_c); \
} while(0)

// 转置: C = A^T
// A: m×n, C: n×m
#define MAT_TRANS(C, A, m, n) do { \
    arm_matrix_instance_f32 _a, _b; \
    arm_mat_init_f32(&_a, m, n, (float *)(A)); \
    arm_mat_init_f32(&_b, n, m, C); \
    arm_mat_trans_f32(&_a, &_b); \
} while(0)

// 按元素加法: C = A + B
#define MAT_ADD(C, A, B, n) do { \
    for (int _i = 0; _i < (n); _i++) (C)[_i] = (A)[_i] + (B)[_i]; \
} while(0)

// 按元素减法: C = A - B
#define MAT_SUB(C, A, B, n) do { \
    for (int _i = 0; _i < (n); _i++) (C)[_i] = (A)[_i] - (B)[_i]; \
} while(0)

// 矩阵求逆: B = A^(-1)
// 注意: arm_mat_inverse_f32 是原地操作，会修改 src
static int mat_inv(float *dst, float *src, int n)
{
    arm_matrix_instance_f32 _a, _b;
    arm_mat_init_f32(&_a, n, n, src);
    arm_mat_init_f32(&_b, n, n, dst);
    if (arm_mat_inverse_f32(&_a, &_b) != ARM_MATH_SUCCESS)
        return -1;
    return 0;
}

/*============================================
 * 回调类型定义
 ============================================*/

typedef void (*KF_Predict_Callback)(struct KF_Instance *kf, float *F, const float *u);
typedef void (*KF_Update_Callback)(struct KF_Instance *kf, float *H, float *h_x, const float *z);

/*============================================
 * KF 实例注册
 ============================================*/

KF_Instance *KF_Register(KF_Init_Config_s *config)
{
    if (!config || config->n == 0 || config->m == 0) return NULL;

    int n = config->n;
    int m = config->m;

    KF_Instance *kf = (KF_Instance *)malloc(sizeof(KF_Instance));
    if (!kf) return NULL;
    memset(kf, 0, sizeof(KF_Instance));

    kf->n = n;
    kf->m = m;
    kf->lambda = (config->lambda > 0 && config->lambda <= 1.0f) ? config->lambda : 1.0f;

    // 分配缓冲区
    kf->x = (float *)malloc(n * sizeof(float));
    kf->P = (float *)malloc(n * n * sizeof(float));
    kf->tmp_n = (float *)malloc(n * sizeof(float));
    kf->tmp_m = (float *)malloc(m * sizeof(float));
    kf->tmp_nn = (float *)malloc(n * n * sizeof(float));
    kf->tmp_nn2 = (float *)malloc(n * n * sizeof(float));
    kf->tmp_nn3 = (float *)malloc(n * n * sizeof(float));
    kf->tmp_mm = (float *)malloc(m * m * sizeof(float));
    kf->tmp_nm = (float *)malloc(n * m * sizeof(float));
    kf->K = (float *)malloc(n * m * sizeof(float));

    if (!kf->x || !kf->P || !kf->tmp_n || !kf->tmp_m ||
        !kf->tmp_nn || !kf->tmp_nn2 || !kf->tmp_nn3 || !kf->tmp_mm || !kf->tmp_nm || !kf->K) {
        KF_Free(kf);
        return NULL;
    }

    // 初始化
    if (config->x_init)
        mat_copy(kf->x, config->x_init, n);
    else
        mat_zero(kf->x, n);

    if (config->P_init)
        mat_copy_nn(kf->P, config->P_init, n);
    else
        mat_eye(kf->P, n);

    return kf;
}

void KF_Free(KF_Instance *kf)
{
    if (!kf) return;
    free(kf->x);
    free(kf->P);
    free(kf->tmp_n);
    free(kf->tmp_m);
    free(kf->tmp_nn);
    free(kf->tmp_nn2);
    free(kf->tmp_nn3);
    free(kf->tmp_mm);
    free(kf->tmp_nm);
    free(kf->K);
    free(kf);
}

void KF_Reset(KF_Instance *kf, const float *x_new, const float *P_new)
{
    if (!kf) return;
    if (x_new)
        mat_copy(kf->x, x_new, kf->n);
    else
        mat_zero(kf->x, kf->n);
    if (P_new)
        mat_copy_nn(kf->P, P_new, kf->n);
    else
        mat_eye(kf->P, kf->n);
}

void KF_Set_Predict_Callback(KF_Instance *kf, KF_Predict_Callback cb)
{
    if (kf) kf->predict_callback = cb;
}

void KF_Set_Update_Callback(KF_Instance *kf, KF_Update_Callback cb)
{
    if (kf) kf->update_callback = cb;
}

/*============================================
 * 线性卡尔曼滤波
 *
 * 预测:
 *   x = F × x
 *   P = F × P × F^T + Q
 *
 * 更新:
 *   y = z - H × x
 *   S = H × P × H^T + R
 *   K = P × H^T × S^(-1)
 *   x = x + K × y
 *   P = (I - K × H) × P
 ============================================*/

void KF_Predict(KF_Instance *kf, const float *F, const float *Q)
{
    if (!kf || !F || !Q) return;
    int n = kf->n;

    // 1. x = F × x
    MAT_MUL(kf->tmp_n, F, kf->x, n, n, 1);
    mat_copy(kf->x, kf->tmp_n, n);

    // 2. P = F × P × F^T + Q
    //    tmp_nn = F × P
    MAT_MUL(kf->tmp_nn, F, kf->P, n, n, n);
    //    tmp_nn3 = F^T (使用独立缓冲区，避免与 F 别名覆盖)
    MAT_TRANS(kf->tmp_nn3, F, n, n);
    //    P = tmp_nn × tmp_nn3 = F×P×F^T
    MAT_MUL(kf->P, kf->tmp_nn, kf->tmp_nn3, n, n, n);

    // 渐消因子
    if (kf->lambda < 1.0f) {
        for (int i = 0; i < n * n; i++)
            kf->P[i] *= kf->lambda;
    }

    // P = P + Q
    MAT_ADD(kf->tmp_nn, kf->P, Q, n * n);
    mat_copy_nn(kf->P, kf->tmp_nn, n);
}

bool KF_Update(KF_Instance *kf, const float *z, const float *H, const float *R)
{
    if (!kf || !z || !H || !R) return false;
    int n = kf->n;
    int m = kf->m;

    // 1. y = z - H × x
    MAT_MUL(kf->tmp_m, H, kf->x, m, n, 1);
    MAT_SUB(kf->tmp_m, z, kf->tmp_m, m);  // tmp_m = y

    // 2. S = H × P × H^T + R
    //    tmp_nm = H × P (m×n)
    MAT_MUL(kf->tmp_nm, H, kf->P, m, n, n);
    //    tmp_nn = H^T (n×m)
    MAT_TRANS(kf->tmp_nn, H, m, n);
    //    tmp_mm = H × P × H^T (m×m)
    MAT_MUL(kf->tmp_mm, kf->tmp_nm, kf->tmp_nn, m, n, m);
    //    tmp_mm = S = tmp_mm + R
    MAT_ADD(kf->tmp_mm, kf->tmp_mm, R, m * m);

    // 3. K = P × H^T × S^(-1)
    //    tmp_nm = P × H^T = P × tmp_nn (n×m)
    MAT_MUL(kf->tmp_nm, kf->P, kf->tmp_nn, n, n, m);
    //    tmp_nn = S^(-1) (m×m)
    mat_eye(kf->tmp_nn, m);
    if (mat_inv(kf->tmp_nn, kf->tmp_mm, m) != 0) return false;
    //    K = PHt × S^(-1)，写入 K 缓冲
    MAT_MUL(kf->K, kf->tmp_nm, kf->tmp_nn, n, m, m);

    // 4. x = x + K × y
    MAT_MUL(kf->tmp_n, kf->K, kf->tmp_m, n, m, 1);
    for (int i = 0; i < n; i++)
        kf->x[i] += kf->tmp_n[i];

    // 5. P = (I - K × H) × P
    MAT_MUL(kf->tmp_nn2, kf->K, H, n, m, n);
    //    tmp_nn = I - K×H
    mat_eye(kf->tmp_nn, n);
    MAT_SUB(kf->tmp_nn, kf->tmp_nn, kf->tmp_nn2, n * n);
    //    P = (I-KH) × P
    //    注意: dst=P, src_B=P 存在覆盖，用 tmp_nn2 中转
    MAT_MUL(kf->tmp_nn2, kf->tmp_nn, kf->P, n, n, n);
    mat_copy_nn(kf->P, kf->tmp_nn2, n);

    return true;
}

/*============================================
 * EKF 预测 (使用回调)
 ============================================*/

void KF_Predict_EKF(KF_Instance *kf, const float *Q, const float *u)
{
    if (!kf || !Q) return;

    if (kf->predict_callback) {
        // 回调计算 F 矩阵
        kf->predict_callback(kf, kf->tmp_nn2, u);
        KF_Predict(kf, kf->tmp_nn2, Q);
    }
}

/*============================================
 * EKF 更新 (使用回调)
 *
 * 回调函数计算:
 *   H: 观测 Jacobian (m×n)
 *   h_x: 预测观测值 (m)
 ============================================*/

bool KF_Update_EKF(KF_Instance *kf, const float *z, const float *R, const float *u)
{
    if (!kf || !z || !R) return false;
    if (!kf->update_callback) return false;

    int n = kf->n;
    int m = kf->m;

    // 回调计算 H 和 h(x)
    // H → tmp_nn (m×n), h_x → tmp_m (m)
    kf->update_callback(kf, kf->tmp_nn, kf->tmp_m, u);

    // 1. y = z - h(x)
    MAT_SUB(kf->tmp_m, z, kf->tmp_m, m);  // tmp_m = y

    // 2. S = H × P × H^T + R
    //    tmp_nm = H × P (m×n)
    MAT_MUL(kf->tmp_nm, kf->tmp_nn, kf->P, m, n, n);
    //    tmp_nn2 = H^T (n×m)
    MAT_TRANS(kf->tmp_nn2, kf->tmp_nn, m, n);
    //    tmp_mm = H × P × H^T (m×m)
    MAT_MUL(kf->tmp_mm, kf->tmp_nm, kf->tmp_nn2, m, n, m);
    //    tmp_mm = S = tmp_mm + R
    MAT_ADD(kf->tmp_mm, kf->tmp_mm, R, m * m);

    // 3. K = P × H^T × S^(-1)
    //    tmp_nm = P × H^T = P × tmp_nn2 (n×m)
    MAT_MUL(kf->tmp_nm, kf->P, kf->tmp_nn2, n, n, m);
    //    tmp_nn2 = S^(-1)，H^T 已不需要可覆盖
    mat_eye(kf->tmp_nn2, m);
    if (mat_inv(kf->tmp_nn2, kf->tmp_mm, m) != 0) return false;
    //    K = PHt × S^(-1)，写入 K 缓冲
    MAT_MUL(kf->K, kf->tmp_nm, kf->tmp_nn2, n, m, m);

    // 4. x = x + K × y
    MAT_MUL(kf->tmp_n, kf->K, kf->tmp_m, n, m, 1);
    for (int i = 0; i < n; i++)
        kf->x[i] += kf->tmp_n[i];

    // 5. P = (I - K × H) × P
    //    H 在 tmp_nn，K 在 kf->K
    MAT_MUL(kf->tmp_nn2, kf->K, kf->tmp_nn, n, m, n);
    mat_eye(kf->tmp_nn, n);
    MAT_SUB(kf->tmp_nn, kf->tmp_nn, kf->tmp_nn2, n * n);
    MAT_MUL(kf->tmp_nn2, kf->tmp_nn, kf->P, n, n, n);
    mat_copy_nn(kf->P, kf->tmp_nn2, n);

    return true;
}

/*============================================
 * 卡方检验
 ============================================*/

float KF_ChiSquare(KF_Instance *kf, const float *z, const float *H)
{
    if (!kf || !z || !H) return -1.0f;
    int n = kf->n;
    int m = kf->m;

    float y[MAT_MAX_N];

    // y = z - H × x
    MAT_MUL(kf->tmp_m, H, kf->x, m, n, 1);
    for (int i = 0; i < m; i++)
        y[i] = z[i] - kf->tmp_m[i];

    // S = H × P × H^T
    MAT_MUL(kf->tmp_nm, H, kf->P, m, n, n);
    MAT_TRANS(kf->tmp_nn, H, m, n);
    MAT_MUL(kf->tmp_mm, kf->tmp_nm, kf->tmp_nn, m, n, m);

    // S^(-1)
    mat_eye(kf->tmp_nn, m);
    if (mat_inv(kf->tmp_nn, kf->tmp_mm, m) != 0) return -1.0f;

    // y^T × S^(-1) × y
    MAT_MUL(kf->tmp_m, kf->tmp_nn, y, m, m, 1);

    float chi = 0;
    for (int i = 0; i < m; i++)
        chi += y[i] * kf->tmp_m[i];

    return chi;
}
