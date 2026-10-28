/**
 * @file    kalman_filter.cpp
 * @brief   通用 KF/EKF 模板成员定义，模型由调用方提供。
 */
#ifndef KALMAN_FILTER_TEMPLATE_IMPLEMENTATION
#define KALMAN_FILTER_TEMPLATE_IMPLEMENTATION

#include "kalman_filter.h"

#include "matrix.hpp"
// ============================== 生命周期 ==============================

template <int _n, int _m>
void KalmanFilter<_n, _m>::init(const Config& config) {
    lambda_ = (config.lambda > 0.0f && config.lambda <= 1.0f) ? config.lambda : 1.0f;

    // 注：C 版误用 mat_zero(x, n) 清零 n×n，越界写入 x 之后的 30 个 float；
    //     此处只清 _n 个（保持“x 清零”本意，消除越界）。
    if (config.x_init) {
        for (int i = 0; i < _n; i++) x_[i][0] = config.x_init[i];
    } else {
        x_ = Matrixf<_n, 1>::zeros();
    }

    if (config.P_init) {
        for (int r = 0; r < _n; r++)
            for (int c = 0; c < _n; c++)
                P_[r][c] = config.P_init[r * _n + c];
    } else {
        P_ = Matrixf<_n, _n>::eye();
    }
}

template <int _n, int _m>
void KalmanFilter<_n, _m>::reset(const float* x_new, const float* P_new) {
    if (x_new) {
        for (int i = 0; i < _n; i++) x_[i][0] = x_new[i];
    } else {
        x_ = Matrixf<_n, 1>::zeros();
    }
    if (P_new) {
        for (int r = 0; r < _n; r++)
            for (int c = 0; c < _n; c++)
                P_[r][c] = P_new[r * _n + c];
    } else {
        P_ = Matrixf<_n, _n>::eye();
    }
}

// ============================== 线性卡尔曼滤波 ==============================

template <int _n, int _m>
void KalmanFilter<_n, _m>::predict(const Matrixf<_n, _n>& F, const Matrixf<_n, _n>& Q) {
    // 1. x = F × x
    x_ = F * x_;
    // 2. P = F × P × F^T
    P_ = F * P_ * F.trans();
    // 渐消因子
    if (lambda_ < 1.0f) P_ *= lambda_;
    // P = P + Q
    P_ += Q;
}

template <int _n, int _m>
bool KalmanFilter<_n, _m>::update(const Matrixf<_m, 1>& z, const Matrixf<_m, _n>& H, const Matrixf<_m, _m>& R) {
    // 1. y = z - H × x
    Matrixf<_m, 1> y = z - H * x_;
    // 2. S = H × P × H^T + R
    Matrixf<_m, _m> S = H * P_ * H.trans() + R;
    // 3. K = P × H^T × S^(-1)
    Matrixf<_m, _m> S_inv = S.inv();
    if (S_inv == Matrixf<_m, _m>::zeros()) return false;   // 奇异
    Matrixf<_n, _m> K = P_ * H.trans() * S_inv;
    // 4. x = x + K × y
    x_ += K * y;
    // 5. P = (I - K × H) × P
    P_ = (Matrixf<_n, _n>::eye() - K * H) * P_;
    return true;
}

// ============================== EKF ==============================

template <int _n, int _m>
void KalmanFilter<_n, _m>::predictEKF(const Matrixf<_n, 1>& predicted_state,
                                    const Matrixf<_n, _n>& F, const Matrixf<_n, _n>& Q) {
    x_ = predicted_state;
    P_ = F * P_ * F.trans();
    if (lambda_ < 1.0f) P_ *= lambda_;
    P_ += Q;
}

template <int _n, int _m>
bool KalmanFilter<_n, _m>::updateEKF(const Matrixf<_m, 1>& z,
                                   const Matrixf<_m, 1>& predicted_measurement,
                                   const Matrixf<_m, _n>& H, const Matrixf<_m, _m>& R) {
    // 1. y = z - h(x)
    Matrixf<_m, 1> y = z - predicted_measurement;
    // 2. H 随状态变化，转置仅在本轮复用，不跨轮缓存。
    const Matrixf<_n, _m> h_trans = H.trans();
    Matrixf<_m, _m> S = H * P_ * h_trans + R;
    // 3. K = P × H^T × S^(-1)
    Matrixf<_m, _m> S_inv = S.inv();
    if (S_inv == Matrixf<_m, _m>::zeros()) return false;
    Matrixf<_n, _m> K = P_ * h_trans * S_inv;
    // 4. x = x + K × y
    x_ += K * y;
    // 5. 保留 (I - K×H)×P 的运算顺序，避免 P-K×(H×P) 的浮点消减差异。
    P_ = (Matrixf<_n, _n>::eye() - K * H) * P_;
    return true;
}

template <int _n, int _m>
template <uint16_t _h_cols>
bool KalmanFilter<_n, _m>::updateEKF(const Matrixf<_m, 1>& z,
                                   const Matrixf<_m, 1>& predicted_measurement,
                                   const Matrixf<_m, _h_cols>& H, const Matrixf<_m, _m>& R) {
    static_assert(_h_cols > 0 && _h_cols < _n, "紧凑 H 必须省略至少一列恒零状态列");
    Matrixf<_m, 1> y = z - predicted_measurement;
    const Matrixf<_h_cols, _m> h_trans = H.trans();

    // 只搬运一次 P 的前几列；包括全部状态行，保留未直接观测状态的交叉协方差。
    Matrixf<_n, _h_cols> p_columns = P_.template block<_n, _h_cols>(0, 0);
    // p_columns 的前 _h_cols 行已连续存放，直接以 CMSIS 视图使用，避免再复制方块。
    const arm_matrix_instance_f32 p_leading = {_h_cols, _h_cols, p_columns[0]};
    Matrixf<_m, _h_cols> hp_leading;
    arm_mat_mult_f32(&H.arm_mat_, &p_leading, &hp_leading.arm_mat_);
    Matrixf<_m, _m> S = hp_leading * h_trans + R;
    Matrixf<_m, _m> S_inv = S.inv();
    if (S_inv == Matrixf<_m, _m>::zeros()) return false;

    const Matrixf<_n, _m> K = p_columns * h_trans * S_inv;
    x_ += K * y;
    // 保留 (I-KH)P，尾部恒零列对应的单位阵列不参与 K×H 及减法。
    const Matrixf<_n, _h_cols> kh = K * H;
    Matrixf<_n, _n> correction = Matrixf<_n, _n>::eye();
    for (int row = 0; row < _n; ++row)
        for (int col = 0; col < _h_cols; ++col)
            correction[row][col] -= kh[row][col];
    P_ = correction * P_;
    return true;
}

// ============================== 卡方检验 ==============================

template <int _n, int _m>
float KalmanFilter<_n, _m>::chiSquare(const Matrixf<_m, 1>& z, const Matrixf<_m, _n>& H) {
    // y = z - H × x
    Matrixf<_m, 1> y = z - H * x_;
    // S = H × P × H^T
    Matrixf<_m, _m> S = H * P_ * H.trans();
    // S^(-1)
    Matrixf<_m, _m> S_inv = S.inv();
    if (S_inv == Matrixf<_m, _m>::zeros()) return -1.0f;
    // y^T × S^(-1) × y
    Matrixf<1, 1> chi = y.trans() * S_inv * y;
    return chi[0][0];
}

#endif // KALMAN_FILTER_TEMPLATE_IMPLEMENTATION
