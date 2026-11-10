/**
 * @file rls.hpp
 * @brief 编译期维数、无堆分配、纯单精度的增量 QR 递推最小二乘。
 * @note 接口源自 leg_main codex/rls 的 e4b09f5；内部使用平方根信息形式与 Givens 旋转。
 */
#pragma once

#include <stdint.h>
#include <math.h>

/** @brief 估计 y = phi^T p；回归量、单位、样本门控和物理约束由调用者负责。 */
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
    bool initialized_ = false;
    float sqrt_forgetting_factor_ = 1.0f;
    float parameters_[_parameters] = {};
    // R^T R 是信息矩阵；R^T z 是加权观测右端项。
    float triangular_[_parameters][_parameters] = {};
    float transformed_observation_[_parameters] = {};
    // 事务暂存区属于对象，不把两个大矩阵压到 RTOS 任务栈上。
    float candidate_parameters_[_parameters] = {};
    float candidate_triangular_[_parameters][_parameters] = {};
    float candidate_observation_[_parameters] = {};
    float row_[_parameters] = {};

public:
    static constexpr uint16_t kParameterCount = _parameters;

    static constexpr uint16_t parameterCount(); ///< 初始化前也返回模板维数。

private:
    static bool validDiagonal(float value);

public:
    /** @brief 初始化；配置非法时保留原状态，配置数组在调用期间有效即可。 */
    bool init(const Config& config);
    /** @brief 保留维数和遗忘因子，重置参数及对角协方差；空指针规则同 Config。 */
    bool reset(const float* parameters = nullptr, const float* covariance_diag = nullptr);
    /** @brief 预测；未初始化、空指针或非有限输入/结果返回 NaN。 */
    float predict(const float* regressor) const;
    /**
     * @brief 同一拍多条标量方程共同更新一个参数向量，遗忘只在本次调用开始应用一次。
     * @param regressors 行优先 [equation_count][kParameterCount]，每行对应一个观测。
     * @param observations [equation_count]；每条方程使用单位观测权重。
     * @note 整组成功后才提交；非法输入或数值失败返回 false，参数和协方差保持原值。
     *       数组长度由调用者保证；同一对象不可并发调用。零回归行仍属于本拍，应用遗忘。
     */
    bool update(const float* regressors, const float* observations, uint16_t equation_count);

    const float* parameters() const; ///< 只读有效参数，未初始化返回 nullptr。
    float covarianceDiagonal(uint16_t index) const; ///< 非法索引、未初始化或计算超出 float 范围返回 NaN；O(n²)。

private:
    bool prepareReset(const float* parameters, const float* covariance_diag);
    void commit();
};

// ============================== 模板成员定义 ==============================

template <int _parameters>
bool RLS<_parameters>::validDiagonal(float value) {
    if (!isfinite(value) || value <= 0.0f)
        return false;
    // 防止逆因子平方已溢出/下溢；相关列引起的协方差放大由查询另行检查。
    const float inverse = 1.0f / value;
    const float variance = inverse * inverse;
    return isfinite(variance) && variance > 0.0f;
}

template <int _parameters>
bool RLS<_parameters>::prepareReset(const float* parameters, const float* covariance_diag) {
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        const float value = parameters ? parameters[i] : 0.0f;
        const float diagonal = covariance_diag ? covariance_diag[i] : 1.0f;
        if (!isfinite(value) || !isfinite(diagonal) || diagonal <= 0.0f)
            return false;
        const float factor = 1.0f / sqrtf(diagonal);
        const float observation = factor * value;
        if (!validDiagonal(factor) || !isfinite(observation))
            return false;
        candidate_parameters_[i] = value;
        candidate_observation_[i] = observation;
        for (uint16_t j = 0; j < kParameterCount; ++j)
            candidate_triangular_[i][j] = i == j ? factor : 0.0f;
    }
    return true;
}

template <int _parameters>
void RLS<_parameters>::commit() {
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        parameters_[i] = candidate_parameters_[i];
        transformed_observation_[i] = candidate_observation_[i];
        for (uint16_t j = 0; j < kParameterCount; ++j)
            triangular_[i][j] = candidate_triangular_[i][j];
    }
}

template <int _parameters>
bool RLS<_parameters>::init(const Config& config) {
    if (!isfinite(config.forgetting_factor) || config.forgetting_factor <= 0.0f ||
        config.forgetting_factor > 1.0f)
        return false;
    if (!prepareReset(config.initial_parameters, config.initial_covariance_diag))
        return false;
    commit();
    initialized_ = true;
    sqrt_forgetting_factor_ = sqrtf(config.forgetting_factor);
    return true;
}

template <int _parameters>
bool RLS<_parameters>::reset(const float* parameters, const float* covariance_diag) {
    if (!initialized_ || !prepareReset(parameters, covariance_diag))
        return false;
    commit();
    return true;
}

template <int _parameters>
float RLS<_parameters>::predict(const float* regressor) const {
    if (!initialized_ || !regressor)
        return NAN;
    float result = 0.0f;
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        if (!isfinite(regressor[i]))
            return NAN;
        result = fmaf(regressor[i], parameters_[i], result);
    }
    return isfinite(result) ? result : NAN;
}

template <int _parameters>
bool RLS<_parameters>::update(const float* regressors, const float* observations, uint16_t equation_count) {
    if (!initialized_ || !regressors || !observations || equation_count == 0)
        return false;
    // 一次调用只遗忘一次：同时缩放 R 和 z，使整个历史目标乘以 lambda。
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        candidate_observation_[i] = transformed_observation_[i] * sqrt_forgetting_factor_;
        if (!isfinite(candidate_observation_[i]))
            return false;
        for (uint16_t j = i; j < kParameterCount; ++j)
            candidate_triangular_[i][j] = triangular_[i][j] * sqrt_forgetting_factor_;
    }
    for (uint16_t row = 0; row < equation_count; ++row) {
        const float* phi = regressors + static_cast<uint32_t>(row) * kParameterCount;
        float observation = observations[row];
        if (!isfinite(observation))
            return false;
        for (uint16_t i = 0; i < kParameterCount; ++i) {
            if (!isfinite(phi[i]))
                return false;
            row_[i] = phi[i];
        }
        for (uint16_t i = 0; i < kParameterCount; ++i) {
            if (row_[i] == 0.0f)
                continue;
            const float diagonal = candidate_triangular_[i][i];
            // 缩放后计算旋转，避免直接计算 a*a+b*b 时溢出或下溢。
            const float scale = fmaxf(fabsf(diagonal), fabsf(row_[i]));
            if (!isfinite(scale) || scale <= 0.0f)
                return false;
            const float a = diagonal / scale;
            const float b = row_[i] / scale;
            const float norm = sqrtf(a * a + b * b);
            const float radius = scale * norm;
            if (!isfinite(radius) || radius <= 0.0f)
                return false;
            const float cosine = a / norm;
            const float sine = b / norm;
            candidate_triangular_[i][i] = radius;
            for (uint16_t j = i + 1; j < kParameterCount; ++j) {
                const float upper = candidate_triangular_[i][j];
                const float lower = row_[j];
                const float rotated_upper = fmaf(sine, lower, cosine * upper);
                const float rotated_lower = fmaf(-sine, upper, cosine * lower);
                if (!isfinite(rotated_upper) || !isfinite(rotated_lower))
                    return false;
                candidate_triangular_[i][j] = rotated_upper;
                row_[j] = rotated_lower;
            }
            const float upper = candidate_observation_[i];
            const float rotated_upper = fmaf(sine, observation, cosine * upper);
            const float rotated_lower = fmaf(-sine, upper, cosine * observation);
            if (!isfinite(rotated_upper) || !isfinite(rotated_lower))
                return false;
            candidate_observation_[i] = rotated_upper;
            observation = rotated_lower;
        }
    }
    // 固定维数的三角回代；没有等待收敛的内层迭代。
    for (uint16_t reverse = kParameterCount; reverse > 0; --reverse) {
        const uint16_t i = reverse - 1;
        const float diagonal = candidate_triangular_[i][i];
        if (!validDiagonal(diagonal))
            return false;
        float value = candidate_observation_[i];
        for (uint16_t j = i + 1; j < kParameterCount; ++j)
            value = fmaf(-candidate_triangular_[i][j], candidate_parameters_[j], value);
        value /= diagonal;
        if (!isfinite(value))
            return false;
        candidate_parameters_[i] = value;
    }
    commit();
    return true;
}

template <int _parameters>
const float* RLS<_parameters>::parameters() const {
    return initialized_ ? parameters_ : nullptr;
}

template <int _parameters>
float RLS<_parameters>::covarianceDiagonal(uint16_t index) const {
    if (!initialized_ || index >= kParameterCount)
        return NAN;
    // P = R^-1 R^-T；解 R^T v = e_index，P[index,index] = v^T v。
    float column[_parameters] = {};
    float variance = 0.0f;
    for (uint16_t i = index; i < kParameterCount; ++i) {
        float value = i == index ? 1.0f : 0.0f;
        for (uint16_t j = index; j < i; ++j)
            value = fmaf(-triangular_[j][i], column[j], value);
        column[i] = value / triangular_[i][i];
        variance = fmaf(column[i], column[i], variance);
        if (!isfinite(column[i]) || !isfinite(variance))
            return NAN;
    }
    return variance > 0.0f ? variance : NAN;
}

template <int _parameters>
constexpr uint16_t RLS<_parameters>::parameterCount() {
    return kParameterCount;
}
