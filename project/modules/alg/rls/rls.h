/**
 * @file rls.h
 * @brief 编译期维数、无堆分配的通用递推最小二乘。
 * @note 移植自 leg_main codex/rls 的 e4b09f5；模板成员定义在 .cpp，由本头文件包含。
 */
#pragma once

#include <stdint.h>

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

// 单独编译实现文件时，不再次包含正在展开的 .cpp（也避免 clangd 的预编译递归诊断）。
#ifndef RLS_TEMPLATE_IMPLEMENTATION
#include "rls.cpp"
#endif
