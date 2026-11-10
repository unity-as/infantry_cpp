/**
 * @file    kalman_filter.hpp
 * @brief   卡尔曼滤波器（线性 KF / EKF）模板类与成员定义
 * @note    仅负责矩阵递推，模型值和 Jacobian 由调用方计算。
 *          类声明与模板成员定义同置于本头文件，不维护固定维数组合。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）
 */
#pragma once

#include "matrix.hpp"

/**
 * @brief 卡尔曼滤波器（线性 KF 与 EKF）
 * @tparam _n 状态维度
 * @tparam _m 观测维度
 */
template <int _n, int _m>
class KalmanFilter {
    static_assert(_n > 0 && _m > 0, "KF 维度必须为正");

public:
    /// 初始化配置
    struct Config {
        const float* x_init;   ///< 初始状态 [_n]，可为 nullptr（默认零向量）
        const float* P_init;   ///< 初始协方差 [_n×_n]，可为 nullptr（默认单位阵）
        float lambda;          ///< 渐消因子 (0~1]，<=0 或 >1 取 1（标准 KF）
    };

    // —— 实例变量 ——
    Matrixf<_n, 1> x_;         ///< 状态向量（列向量）
    Matrixf<_n, _n> P_;        ///< 协方差矩阵

private:
    float lambda_ = 1.0f;                        ///< 渐消因子

    // —— static 变量 ——
public:
    static constexpr int kStateDim = _n;
    static constexpr int kObsDim   = _m;

    // —— 实例函数 ——
    void init(const Config& config);                     ///< 替代 KF_Register（无堆，无需 free）
    void reset(const float* x_new, const float* P_new);  ///< 替代 KF_Reset
    void predict(const Matrixf<_n, _n>& F, const Matrixf<_n, _n>& Q);          ///< 线性预测
    bool update(const Matrixf<_m, 1>& z, const Matrixf<_m, _n>& H, const Matrixf<_m, _m>& R);  ///< 线性更新
    /// predicted_state=f(x,u)，F=df/dx；两者均由外部在预测前的状态处计算。
    void predictEKF(const Matrixf<_n, 1>& predicted_state,
                    const Matrixf<_n, _n>& F, const Matrixf<_n, _n>& Q);
    /// predicted_measurement=h(x)，H=dh/dx；均在预测成功后的先验状态处计算。
    bool updateEKF(const Matrixf<_m, 1>& z, const Matrixf<_m, 1>& predicted_measurement,
                   const Matrixf<_m, _n>& H, const Matrixf<_m, _m>& R);
    /// 紧凑 H 仅存前 _h_cols 列，其余列恒为零；仍更新全部状态及协方差交叉项。
    template <uint16_t _h_cols>
    bool updateEKF(const Matrixf<_m, 1>& z, const Matrixf<_m, 1>& predicted_measurement,
                   const Matrixf<_m, _h_cols>& H, const Matrixf<_m, _m>& R);
};

// ============================== 生命周期 ==============================

template <int _n, int _m>
void KalmanFilter<_n, _m>::init(const Config& config) {
    lambda_ = (config.lambda > 0.0f && config.lambda <= 1.0f) ? config.lambda : 1.0f;
    reset(config.x_init, config.P_init);
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
