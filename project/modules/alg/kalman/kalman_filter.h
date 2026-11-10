/**
 * @file    kalman_filter.h
 * @brief   卡尔曼滤波器（线性 KF / EKF）模板类声明
 * @note    仅负责矩阵递推，模型值和 Jacobian 由调用方计算。
 *          模板成员定义在 .cpp，由本头文件包含，不维护固定维数组合。
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
    float chiSquare(const Matrixf<_m, 1>& z, const Matrixf<_m, _n>& H);       ///< 卡方检验
};

// 单独编译实现文件时，不再次包含正在展开的 .cpp（也避免 clangd 的预编译递归诊断）。
#ifndef KALMAN_FILTER_TEMPLATE_IMPLEMENTATION
#include "kalman_filter.cpp"
#endif
