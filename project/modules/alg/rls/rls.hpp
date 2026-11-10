/**
 * @file rls.hpp
 * @brief 编译期维数、无堆分配、纯单精度的增量 QR 递推最小二乘。
 * @note 平方根信息形式，使用 Givens 旋转和三角回代。
 */
#pragma once

#include <stdint.h>
#include <math.h>

/** @brief 估计 y = phi^T p；样本有效性、尺度和物理约束由调用者负责。 */
template <int _parameters>
class RLS {
    static_assert(_parameters > 0 && _parameters <= UINT16_MAX,
                  "RLS 参数数必须为正且能由 uint16_t 表示");
public:
    struct Config {
        float forgetting_factor = 1.0f;           ///< 每次 update 的遗忘因子，(0, 1]
        const float* initial_parameters = nullptr; ///< [_parameters]，空指针表示零初值
        const float* initial_covariance_diag = nullptr; ///< 严格正对角初值，空指针表示单位阵
    };

private:
    float sqrt_forgetting_factor_ = 1.0f;
    float parameters_[_parameters] = {};
    // R^T R 是信息矩阵；R^T z 是加权观测右端项。
    float triangular_[_parameters][_parameters] = {};
    float transformed_observation_[_parameters] = {};
    float row_[_parameters] = {}; ///< Givens 旋转消元工作行。

public:
    static constexpr uint16_t kParameterCount = _parameters;

    static constexpr uint16_t parameterCount();

    /** @brief 设置遗忘因子与先验；调用者保证配置有效。 */
    void init(const Config& config);
    /** @brief 保留维数和遗忘因子，重置参数及对角协方差；空指针规则同 Config。 */
    void reset(const float* parameters = nullptr, const float* covariance_diag = nullptr);
    float predict(const float* regressor) const;
    /**
     * @brief 同一拍多条标量方程共同更新一个参数向量，遗忘只在本次调用开始应用一次。
     * @param regressors 行优先 [equation_count][kParameterCount]，每行对应一个观测。
     * @param observations [equation_count]；每条方程使用单位观测权重。
     * @pre 已初始化，样本有限、数组长度正确、equation_count > 0；同一对象不可并发调用。
     */
    void update(const float* regressors, const float* observations, uint16_t equation_count);

    const float* parameters() const;
    float covarianceDiagonal(uint16_t index) const; ///< index < kParameterCount；O(n²)。
};

// ============================== 模板成员定义 ==============================

template <int _parameters>
void RLS<_parameters>::init(const Config& config) {
    sqrt_forgetting_factor_ = sqrtf(config.forgetting_factor);
    reset(config.initial_parameters, config.initial_covariance_diag);
}

template <int _parameters>
void RLS<_parameters>::reset(const float* parameters, const float* covariance_diag) {
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        const float value = parameters ? parameters[i] : 0.0f;
        const float diagonal = covariance_diag ? covariance_diag[i] : 1.0f;
        const float factor = 1.0f / sqrtf(diagonal);
        parameters_[i] = value;
        transformed_observation_[i] = factor * value;
        for (uint16_t j = 0; j < kParameterCount; ++j)
            triangular_[i][j] = i == j ? factor : 0.0f;
    }
}

template <int _parameters>
float RLS<_parameters>::predict(const float* regressor) const {
    float result = 0.0f;
    for (uint16_t i = 0; i < kParameterCount; ++i)
        result = fmaf(regressor[i], parameters_[i], result);
    return result;
}

template <int _parameters>
void RLS<_parameters>::update(const float* regressors, const float* observations, uint16_t equation_count) {
    // 一次调用只遗忘一次：同时缩放 R 和 z，使整个历史目标乘以 lambda。
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        transformed_observation_[i] *= sqrt_forgetting_factor_;
        for (uint16_t j = i; j < kParameterCount; ++j)
            triangular_[i][j] *= sqrt_forgetting_factor_;
    }
    for (uint16_t row = 0; row < equation_count; ++row) {
        const float* phi = regressors + static_cast<uint32_t>(row) * kParameterCount;
        float observation = observations[row];
        for (uint16_t i = 0; i < kParameterCount; ++i)
            row_[i] = phi[i];
        for (uint16_t i = 0; i < kParameterCount; ++i) {
            if (row_[i] == 0.0f)
                continue;
            const float diagonal = triangular_[i][i];
            // 缩放后计算旋转，避免直接计算 a*a+b*b 时溢出或下溢。
            const float scale = fmaxf(fabsf(diagonal), fabsf(row_[i]));
            const float a = diagonal / scale;
            const float b = row_[i] / scale;
            const float norm = sqrtf(a * a + b * b);
            const float cosine = a / norm;
            const float sine = b / norm;
            triangular_[i][i] = scale * norm;
            for (uint16_t j = i + 1; j < kParameterCount; ++j) {
                const float upper = triangular_[i][j];
                const float lower = row_[j];
                triangular_[i][j] = fmaf(sine, lower, cosine * upper);
                row_[j] = fmaf(-sine, upper, cosine * lower);
            }
            const float upper = transformed_observation_[i];
            transformed_observation_[i] = fmaf(sine, observation, cosine * upper);
            observation = fmaf(-sine, upper, cosine * observation);
        }
    }
    // 固定维数的三角回代；没有等待收敛的内层迭代。
    for (uint16_t reverse = kParameterCount; reverse > 0; --reverse) {
        const uint16_t i = reverse - 1;
        float value = transformed_observation_[i];
        for (uint16_t j = i + 1; j < kParameterCount; ++j)
            value = fmaf(-triangular_[i][j], parameters_[j], value);
        parameters_[i] = value / triangular_[i][i];
    }
}

template <int _parameters>
const float* RLS<_parameters>::parameters() const {
    return parameters_;
}

template <int _parameters>
float RLS<_parameters>::covarianceDiagonal(uint16_t index) const {
    // P = R^-1 R^-T；解 R^T v = e_index，P[index,index] = v^T v。
    float column[_parameters] = {};
    float variance = 0.0f;
    for (uint16_t i = index; i < kParameterCount; ++i) {
        float value = i == index ? 1.0f : 0.0f;
        for (uint16_t j = index; j < i; ++j)
            value = fmaf(-triangular_[j][i], column[j], value);
        column[i] = value / triangular_[i][i];
        variance = fmaf(column[i], column[i], variance);
    }
    return variance;
}

template <int _parameters>
constexpr uint16_t RLS<_parameters>::parameterCount() {
    return kParameterCount;
}
