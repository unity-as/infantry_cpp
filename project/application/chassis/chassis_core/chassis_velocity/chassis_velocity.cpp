/**
 * @file    chassis_velocity.cpp
 * @brief   底盘四轮速度环 + 功率控制（C → C++）
 * @note    从 C 版 chassis_velocity 迁移，功率限流后的积分反算由 PID 模块处理，禁堆。
 */
#include "chassis_velocity.h"

void ChassisVelocity::init(const Config& config)
{
    power_.init(config.power);

    // 注册四轮电机 + 速度环，轮序 [rf, lf, lr, rr]
    DJIMotor::Config m_cfg = {};
    m_cfg.can_handle = config.can_handle;
    m_cfg.motor_type = DJIMotor_3508;
    m_cfg.reduction_ratio = config.reduction_ratio > 0.0f ? config.reduction_ratio : 1.0f;
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
    float speed[CHASSIS_WHEEL_NUM], feedback_current[CHASSIS_WHEEL_NUM];
    for (int i = 0; i < CHASSIS_WHEEL_NUM; ++i) {
        speed[i] = motor_[i].velocity_;
        feedback_current[i] = motor_[i].current_;
    }

    // 速度环 → 预测功率 → 缩放 → 限幅发电流 → 反算：仅使能时跑
    if (enable_) {
        // 1. 速度环：目标速度 → I_raw
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
            pid_velocity_[i].setSetpoint(target_omega_[i]);
            pid_velocity_[i].update(motor_[i].velocity_);
            current_raw_[i] = pid_velocity_[i].output_;
        }

        // 2. 功率管理返回单轮限幅、总功率限幅后的最终电流。
        float limited_current[CHASSIS_WHEEL_NUM];
        power_.limit(current_raw_, speed, limited_current);

        // 4. 写入限流后的电流，并由 PID 内部处理积分反算
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++) {
            motor_[i].setCurrent(limited_current[i]);
            pid_velocity_[i].backCalcAntiWindup(limited_current[i]);
        }
    }

    power_total_ = power_.getPower(feedback_current, speed);
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
    }
}

void ChassisVelocity::setPowerLimit(float limit)
{
    power_.setPowerLimit(limit);
}

float ChassisVelocity::getPower()
{
    return power_total_;
}
