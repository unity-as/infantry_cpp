/**
 * @file rls.hpp
 * @brief 编译期维数、无堆分配的通用递推最小二乘。
 * @note 移植自 leg_main codex/rls 的 e4b09f5；仅整理模板文件与类声明布局。
 */
#pragma once

#include <stdint.h>
#include <float.h>
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
    float forgetting_factor_ = 1.0f;
    float parameters_[_parameters] = {};
    float covariance_[_parameters][_parameters] = {};
    // 事务暂存区属于对象，不把两个大矩阵压到 RTOS 任务栈上。
    float candidate_parameters_[_parameters] = {};
    float candidate_covariance_[_parameters][_parameters] = {};
    double projection_[_parameters] = {};

public:
    static constexpr uint16_t kParameterCount = _parameters;

    static constexpr uint16_t parameterCount(); ///< 初始化前也返回模板维数。

private:
    static bool fitsFloat(double value);

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
    float covarianceDiagonal(uint16_t index) const; ///< 非法索引/未初始化返回 NaN。

private:
    bool prepareReset(const float* parameters, const float* covariance_diag);
    void commit();
};

template <int _parameters>
bool RLS<_parameters>::fitsFloat(double value) {
    return isfinite(value) && value <= FLT_MAX && value >= -FLT_MAX;
}

template <int _parameters>
bool RLS<_parameters>::prepareReset(const float* parameters, const float* covariance_diag) {
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        const float value = parameters ? parameters[i] : 0.0f;
        const float diagonal = covariance_diag ? covariance_diag[i] : 1.0f;
        if (!isfinite(value) || !isfinite(diagonal) || diagonal <= 0.0f)
            return false;
        candidate_parameters_[i] = value;
        for (uint16_t j = 0; j < kParameterCount; ++j)
            candidate_covariance_[i][j] = i == j ? diagonal : 0.0f;
    }
    return true;
}

template <int _parameters>
void RLS<_parameters>::commit() {
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        parameters_[i] = candidate_parameters_[i];
        for (uint16_t j = 0; j < kParameterCount; ++j)
            covariance_[i][j] = candidate_covariance_[i][j];
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
    forgetting_factor_ = config.forgetting_factor;
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
    double result = 0.0;
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        if (!isfinite(regressor[i]))
            return NAN;
        result += static_cast<double>(regressor[i]) * parameters_[i];
    }
    return fitsFloat(result) ? static_cast<float>(result) : NAN;
}

template <int _parameters>
bool RLS<_parameters>::update(const float* regressors, const float* observations, uint16_t equation_count) {
    if (!initialized_ || !regressors || !observations || equation_count == 0)
        return false;
    for (uint16_t i = 0; i < kParameterCount; ++i) {
        candidate_parameters_[i] = parameters_[i];
        for (uint16_t j = 0; j < kParameterCount; ++j) {
            const double value = static_cast<double>(covariance_[i][j]) / forgetting_factor_;
            if (!fitsFloat(value))
                return false;
            candidate_covariance_[i][j] = static_cast<float>(value);
        }
    }
    for (uint16_t row = 0; row < equation_count; ++row) {
        const float* phi = regressors + static_cast<uint32_t>(row) * kParameterCount;
        if (!isfinite(observations[row]))
            return false;
        for (uint16_t i = 0; i < kParameterCount; ++i) {
            if (!isfinite(phi[i]))
                return false;
        }
        double prediction = 0.0;
        double denominator = 1.0;
        for (uint16_t i = 0; i < kParameterCount; ++i) {
            projection_[i] = 0.0;
            for (uint16_t j = 0; j < kParameterCount; ++j)
                projection_[i] += static_cast<double>(candidate_covariance_[i][j]) * phi[j];
            denominator += static_cast<double>(phi[i]) * projection_[i];
            prediction += static_cast<double>(phi[i]) * candidate_parameters_[i];
        }
        if (!isfinite(denominator) || denominator < 1.0 || !isfinite(prediction))
            return false;
        const double error = static_cast<double>(observations[row]) - prediction;
        for (uint16_t i = 0; i < kParameterCount; ++i) {
            const double value = candidate_parameters_[i] + projection_[i] / denominator * error;
            if (!fitsFloat(value))
                return false;
            candidate_parameters_[i] = static_cast<float>(value);
        }
        // 对称秩一更新，只计算上三角并镜像；中间量用 double 降低相消误差。
        for (uint16_t i = 0; i < kParameterCount; ++i) {
            for (uint16_t j = i; j < kParameterCount; ++j) {
                const double value = candidate_covariance_[i][j] -
                    projection_[i] * projection_[j] / denominator;
                if (!fitsFloat(value) || (i == j && value <= 0.0))
                    return false;
                candidate_covariance_[i][j] = candidate_covariance_[j][i] = static_cast<float>(value);
                if (i == j && candidate_covariance_[i][i] <= 0.0f)
                    return false;
            }
        }
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
    return initialized_ && index < kParameterCount ? covariance_[index][index] : NAN;
}

template <int _parameters>
constexpr uint16_t RLS<_parameters>::parameterCount() {
    return kParameterCount;
}
