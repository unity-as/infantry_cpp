/**
 * @file    chassis_velocity.cpp
 * @brief   底盘四轮速度环 + 功率控制（C → C++）
 * @note    从 C 版 chassis_velocity 迁移，逻辑不变，禁堆。
 */
#include "chassis_velocity.h"

// 反算抗饱和：功率控制限幅后回灌积分，integral -= (1-scale)*I_raw*coef
void ChassisVelocity::backCalc(float scale)
{
    if (scale >= 1.0f) return;   // 未限流，无需反算

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        float back = (1.0f - scale) * current_raw_[i] * back_calc_coef_;
        pid_velocity_[i].setIntegral(pid_velocity_[i].integral_ - back);
    }
}

void ChassisVelocity::init(const Config& config)
{
    back_calc_coef_ = config.back_calc_coef;
    power_limit_ = 0.0f;   // 默认不限流，应用层覆盖
    scale_ = 1.0f;

    // 注册四轮电机 + 速度环，轮序 [rf, lf, lr, rr]
    DJIMotor::Config m_cfg = {
        .can_handle = config.can_handle,
        .motor_type = DJIMotor_3508,
        .reduction_ratio = config.reduction_ratio > 0.0f ? config.reduction_ratio : 1.0f,
    };
    uint8_t motor_id[CHASSIS_WHEEL_NUM] = {
        config.motor_id_rf, config.motor_id_lf, config.motor_id_lr, config.motor_id_rr
    };
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
        m_cfg.motor_id = motor_id[i];
        motor_[i].init(m_cfg);
        pid_velocity_[i].init(config.pid_velocity);
    }
}

void ChassisVelocity::setTarget(const float omega_deg[CHASSIS_WHEEL_NUM])
{
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        target_omega_[i] = omega_deg[i];
}

void ChassisVelocity::tick()
{
    float scale = scale_;   // 失能时保持上一次缩放比（current_raw 已在 setEnable 清 0）

    // 速度环 → 预测功率 → 缩放 → 限幅发电流 → 反算：仅使能时跑
    if (enable_) {
        // 1. 速度环：目标速度 → I_raw
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
            pid_velocity_[i].setSetpoint(target_omega_[i]);
            pid_velocity_[i].update(motor_[i].velocity_);
            current_raw_[i] = pid_velocity_[i].output_;
        }

        // 2. 预测功率：P = Σ Kt·I_raw·ω（负功率/倒灌算 0）
        float power_pred = 0.0f;
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
            power_pred += fmaxf(0.0f, CHASSIS_POWER_COEF * current_raw_[i] * motor_[i].velocity_);

        // 3. 缩放比：limit<=0 放开
        scale = 1.0f;
        if (power_limit_ > 0.0f && power_pred > power_limit_)
            scale = power_limit_ / power_pred;
        if (scale > 1.0f) scale = 1.0f;
        if (scale < 0.0f) scale = 0.0f;
        scale_ = scale;

        // 4. 限幅电流 + 发电流
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
            motor_[i].setCurrent(scale * current_raw_[i]);

        // 5. 反算抗饱和
        backCalc(scale);
    }

    // 6. 实测总功率（监测）—— 不受使能影响，持续跑
    power_total_ = CHASSIS_GET_POWER(&motor_[0])
                 + CHASSIS_GET_POWER(&motor_[1])
                 + CHASSIS_GET_POWER(&motor_[2])
                 + CHASSIS_GET_POWER(&motor_[3]);

    //DEBUG
    static float max_power = 0.0f;
    static float power_out = 0.0f;
    static float max_outpower = 0.0f;
    power_out = fmaxf(0.0f, CHASSIS_POWER_COEF * scale * current_raw_[0] * motor_[0].velocity_)
              + fmaxf(0.0f, CHASSIS_POWER_COEF * scale * current_raw_[1] * motor_[1].velocity_)
              + fmaxf(0.0f, CHASSIS_POWER_COEF * scale * current_raw_[2] * motor_[2].velocity_)
              + fmaxf(0.0f, CHASSIS_POWER_COEF * scale * current_raw_[3] * motor_[3].velocity_);
    if (power_out > max_outpower)
        max_outpower = power_out;
    if (power_total_ > max_power)
        max_power = power_total_;
}

void ChassisVelocity::setEnable(uint8_t enable)
{
    enable_ = enable ? 1 : 0;
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        motor_[i].setEnable(enable_);

    // 失能时清一次：复位速度环 PID + 清中间量，防停转期间积分饱和、使能瞬间冲电流
    if (!enable_) {
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
            pid_velocity_[i].reset();
            current_raw_[i] = 0.0f;
        }
        scale_ = 1.0f;
    }
}

void ChassisVelocity::setPowerLimit(float limit)
{
    power_limit_ = limit;
}

float ChassisVelocity::getPower()
{
    return power_total_;
}
