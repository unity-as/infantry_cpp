/**
 * @file    chassis_motion.cpp
 * @brief   底盘运动学解算（C → C++）
 * @note    从 C 版 chassis_motion 迁移，逻辑不变，禁堆。
 */
#include "chassis_motion.h"
#include <math.h>

#define SQRT2  1.41421356f
#define RAD_TO_DEG  57.2958f

// 运动学解算：底盘速度(v, θ, w) → 四轮目标速度，派发给 [2]
void ChassisMotion::update()
{
    //移动分量
    float x_group = v_ * (cosf(theta_) - sinf(theta_)) * move_scale_;
    float y_group = v_ * (sinf(theta_) + cosf(theta_)) * move_scale_;
    //旋转分量
    float wheel_rot = w_rot_ * rotate_scale_;

    //四轮目标速度（rad/s），轮序 [rf, lf, lr, rr]
    float omega_rf =  y_group + wheel_rot;
    float omega_lf = -x_group + wheel_rot;
    float omega_lr = -y_group + wheel_rot;
    float omega_rr =  x_group + wheel_rot;

    //全向取反 + 转 deg/s，派发给 [2] 速度环
    float omega_deg[CHASSIS_WHEEL_NUM] = {
        -omega_rf * RAD_TO_DEG,
        -omega_lf * RAD_TO_DEG,
        -omega_lr * RAD_TO_DEG,
        -omega_rr * RAD_TO_DEG
    };
    velocity_.setTarget(omega_deg);
}

void ChassisMotion::timCallback(void* device)
{
    ChassisMotion* motion = static_cast<ChassisMotion*>(device);
    motion->update();           // 运动学 → 派目标速度
    motion->velocity_.tick();   // 速度环 + 功率 + 发电流
}

void ChassisMotion::init(const Config& config)
{
    float L, W;
    if (config.D > 0.0f) {
        L = config.D * SQRT2;
        W = L;
    } else if (config.L > 0.0f && config.W > 0.0f) {
        L = config.L;
        W = config.W;
    } else {
        return;
    }

    move_scale_ = 1.0f / (SQRT2 * config.r);
    rotate_scale_ = (L + W) / (2.0f * SQRT2 * config.r);
    if (config.type == Type::Mecanum) {
        move_scale_ *= SQRT2;
        rotate_scale_ *= SQRT2;
    }

    // 创建 [2] 速度环+功率（四轮电机）
    velocity_.init(config.velocity);

    // 注册 1kHz 定时器（先于 dji_motor，同一拍先算电流再发送）
    tim_.setCallback(timCallback, this);
    TIM::Config tim_cfg = { .htim = config.tim_handle };
    tim_.init(tim_cfg);
}
