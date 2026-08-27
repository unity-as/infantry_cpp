#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>
#include <stdbool.h>

/*============================================
 * KF Instance - 卡尔曼滤波器实例
 *
 * 缓冲区管理原则: 每个中间结果有独立存储，避免别名覆盖
 *
 * 状态向量: x [n]
 * 协方差矩阵: P [n×n]
 * 工作缓冲区:
 *   - tmp_n [n]: F×x, x+K×y 等向量结果
 *   - tmp_m [m]: H×x, y=z-H×x 等观测残差
 *   - tmp_nn [n×n]: F×P, F^T, S^(-1), I-KH 等矩阵
 *   - tmp_nn2 [n×n]: KH, (I-KH)×P 等中间矩阵
 *   - tmp_mm [m×m]: S = H×P×H^T+R
 *   - tmp_nm [n×m]: PHt, K 等卡尔曼增益相关
 ============================================*/

typedef struct KF_Instance {
    int n;                              // 状态维度
    int m;                              // 观测维度
    float *x;                           // 状态向量 [n]
    float *P;                           // 协方差矩阵 [n×n]

    // 用户回调 (EKF/UKF 用)
    // predict_callback: 计算 F 矩阵 (状态转移 Jacobian)
    //   kf: KF实例
    //   F: 输出 F 矩阵 [n×n]
    //   u: 控制输入 (可为 NULL)
    void (*predict_callback)(struct KF_Instance *kf, float *F, const float *u);

    // update_callback: 计算 H 矩阵 (观测 Jacobian) 和 h(x) (预测观测)
    //   kf: KF实例
    //   H: 输出 H 矩阵 [m×n]
    //   h_x: 输出 h(x) 向量 [m] (预测的观测值)
    //   z: 实际观测值 [m] (用于参考，可不用)
    void (*update_callback)(struct KF_Instance *kf, float *H, float *h_x, const float *z);

    float lambda;                       // 渐消因子 (0~1, 1=标准KF)

    // 内部工作缓冲区
    float *tmp_n;                       // [n] 向量计算
    float *tmp_m;                       // [m] 向量计算
    float *tmp_nn;                      // [n×n] 矩阵计算
    float *tmp_nn2;                     // [n×n] 矩阵计算
    float *tmp_nn3;                     // [n×n] 矩阵计算 (F^T 专用)
    float *tmp_mm;                      // [m×m] 矩阵计算
    float *tmp_nm;                      // [n×m] PHt, 临时计算
    float *K;                           // [n×m] 卡尔曼增益
} KF_Instance;

/*============================================
 * 初始化配置
 ============================================*/

typedef struct {
    int n;                              // 状态维度
    int m;                              // 观测维度
    const float *x_init;                // 初始状态 [n], 可为NULL
    const float *P_init;                // 初始协方差 [n×n], 可为NULL
    float lambda;                       // 渐消因子 (0~1), 0=标准KF
} KF_Init_Config_s;

/*============================================
 * API
 ============================================*/

KF_Instance *KF_Register(KF_Init_Config_s *config);
void KF_Free(KF_Instance *kf);
void KF_Reset(KF_Instance *kf, const float *x_new, const float *P_new);

// 设置回调 (用于 EKF)
void KF_Set_Predict_Callback(KF_Instance *kf, void (*cb)(struct KF_Instance*, float*, const float*));
void KF_Set_Update_Callback(KF_Instance *kf, void (*cb)(struct KF_Instance*, float*, float*, const float*));

// 线性卡尔曼滤波
void KF_Predict(KF_Instance *kf, const float *F, const float *Q);
bool KF_Update(KF_Instance *kf, const float *z, const float *H, const float *R);

// EKF: 预测步 (用户通过回调提供F矩阵)
void KF_Predict_EKF(KF_Instance *kf, const float *Q, const float *u);

// EKF: 更新步 (用户通过回调提供H矩阵和h(x))
bool KF_Update_EKF(KF_Instance *kf, const float *z, const float *R, const float *u);

#endif // KALMAN_FILTER_H
