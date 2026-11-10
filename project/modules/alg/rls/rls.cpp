/** @file rls.cpp @brief 通用 RLS 模板成员定义。 */
#ifndef RLS_TEMPLATE_IMPLEMENTATION
#define RLS_TEMPLATE_IMPLEMENTATION

#include "rls.h"

#include <float.h>
#include <math.h>

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

#endif // RLS_TEMPLATE_IMPLEMENTATION
