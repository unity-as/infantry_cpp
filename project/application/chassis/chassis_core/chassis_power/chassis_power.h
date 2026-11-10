/** @file chassis_power.h @brief 四轮功率模型及固定次数解析电流限流。 */
#pragma once

#include "super_cap.h"
#include "rls.hpp"

class ChassisPower {
public:
    struct Config {
        float torque_constant;   ///< 输出轴 N·m/A，与输出轴转速配套。
        float speed_loss;        ///< k_omega，W/(rad/s)，非负。
        float torque_loss;       ///< k_tau，W/(N·m)^2，非负。
        float idle_power_w;      ///< 四轮及所覆盖固定负载的总静态损耗，W，非负。
        float current_limit_a;   ///< 单轮电流上限，A，正值且不超过驱动范围。
        float forgetting_factor = 0.9999f; ///< 每个有效新功率样本遗忘一次。
        SuperCap* super_cap = nullptr;
    };

private:
    Config config_ = {};
    float power_limit_w_ = 0.0f;
    RLS<2> model_;

public:
    void init(const Config& config);
    /// 非正预算禁止驱动电流，不再以零值表达不限流。
    void setPowerLimit(float limit_w);
    /// 数组均为四轮；转速为输出轴 deg/s，电流为 A。输入须有限，配置须物理有效。
    void limit(const float* requested_a, const float* speed_deg_s, float* limited_a) const;
    /// 保守模型：负机械功率不抵扣其他轮耗电；未确认回馈能力前不借用制动功率。
    float predict(const float* current_a, const float* speed_deg_s) const;
    /// 仅在有新功率反馈时调用；电流/转速须为与该功率同一时窗的实际反馈。
    /// 没有样本就不调用，模型保留上次参数；当前协议尚未接入，不自动拿异步最新值训练。
    void updateModel(float measured_power_w, const float* feedback_a, const float* speed_deg_s);
    /// 有有效超电反馈时返回实测值，否则用当前模型估算；不据此提高功率预算。
    float getPower(const float* feedback_a, const float* speed_deg_s) const;
};
