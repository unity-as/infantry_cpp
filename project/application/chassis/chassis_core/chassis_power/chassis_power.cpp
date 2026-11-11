#include "chassis_power.h"
#include <math.h>

void ChassisPower::init(const Config& config) {
    config_ = config;
    power_limit_w_ = 0.0f;
    const float initial[] = {config.speed_loss, config.torque_loss};
    model_.init({.forgetting_factor = config.forgetting_factor, .initial_parameters = initial});
}

void ChassisPower::setPowerLimit(float limit_w) {
    power_limit_w_ = fmaxf(0.0f, limit_w);
}

float ChassisPower::predict(const float* current_a, const float* speed_deg_s) const {
    float power = config_.idle_power_w;
    for (int i = 0; i < 4; ++i) {
        const float omega = speed_deg_s[i] * 0.01745329252f;
        const float torque = current_a[i] * config_.torque_constant;
        power += fmaxf(0.0f, torque * omega) + fmaxf(0.0f, model_.parameters()[0]) * fabsf(omega)
               + fmaxf(0.0f, model_.parameters()[1]) * torque * torque;
    }
    return power;
}

void ChassisPower::limit(const float* requested_a, const float* speed_deg_s, float* limited_a) const {
    float a = 0.0f, b = 0.0f, c = config_.idle_power_w;
    for (int i = 0; i < 4; ++i) {
        limited_a[i] = fminf(config_.current_limit_a, fmaxf(-config_.current_limit_a, requested_a[i]));
        const float torque = limited_a[i] * config_.torque_constant;
        const float omega = speed_deg_s[i] * 0.01745329252f;
        a += fmaxf(0.0f, model_.parameters()[1]) * torque * torque;
        b += fmaxf(0.0f, torque * omega);
        c += fmaxf(0.0f, model_.parameters()[0]) * fabsf(omega);
    }
    // P(alpha)=a*alpha^2+b*alpha+c，三项非负，在 [0,1] 单调。
    float scale = 1.0f;
    const float available = power_limit_w_ - c;
    if (power_limit_w_ <= 0.0f || available <= 0.0f) {
        scale = 0.0f;  // c 已超预算时无可行解；零电流也不代表实际功率已达标。
    } else if (a + b > available) {
        if (a > 0.0f) {
            // 正根的有理化形式，避免 sqrt(b*b+...)-b 相减消减。
            scale = 2.0f * available / (b + sqrtf(b * b + 4.0f * a * available));
        } else {
            scale = available / b;
        }
    }
    for (int i = 0; i < 4; ++i) limited_a[i] *= scale;
}

void ChassisPower::updateModel(float measured_power_w, const float* feedback_a, const float* speed_deg_s) {
    if (!isfinite(measured_power_w) || measured_power_w < 0.0f) return;
    float phi[2] = {}, mechanical = 0.0f;
    for (int i = 0; i < 4; ++i) {
        if (!isfinite(feedback_a[i]) || !isfinite(speed_deg_s[i])) return;
        const float torque = feedback_a[i] * config_.torque_constant;
        const float omega = speed_deg_s[i] * 0.01745329252f;
        // 回馈能力及测量符号尚未确认，制动样本暂不用于辨识。
        if (torque * omega < 0.0f) return;
        mechanical += torque * omega;
        phi[0] += fabsf(omega);
        phi[1] += torque * torque;
    }
    if (phi[0] == 0.0f && phi[1] == 0.0f) return;
    const float observation = measured_power_w - mechanical - config_.idle_power_w;
    model_.update(phi, &observation, 1);
}

float ChassisPower::getPower(const float* feedback_a, const float* speed_deg_s) const {
    if (config_.super_cap && config_.super_cap->isOnline())
        return config_.super_cap->feedback().output_power_w;
    return predict(feedback_a, speed_deg_s);
}
