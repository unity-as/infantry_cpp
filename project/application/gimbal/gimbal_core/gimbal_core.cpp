/**
 * @file    gimbal_core.cpp
 * @brief   云台双轴串级 PID 控制核心（C → C++）
 * @note    从 C 版 gimbal_core 迁移，逻辑不变，禁堆。
 */
#include "gimbal_core.h"
#include "bsp_tim.h"

#define RAD_TO_DEG          57.2957795f

Gimbal* Gimbal::instances_[GIMBAL_MAX] = {nullptr};
uint8_t Gimbal::idx_ = 0;
TIM Gimbal::tim_;

/*============================================
 *  单轴串级 PID 更新
 *  内置 motor enable/valid 检查，上层无需关心
 ============================================*/

void Gimbal::axisUpdate(Axis& axis)
{
    if (!axis.motor.motor_enable_ || !axis.motor.motor_valid_)
        return;

    // 位置模式: 位置环 → vel_target
    if (axis.mode == Mode::Position) {
        if (++(axis.div_cnt) >= axis.pos_freq_div) {
            axis.div_cnt = 0;
            axis.pid_pos.setSetpoint(axis.target);
            axis.pid_pos.update(axis.actual_angle);
            axis.vel_target = axis.pid_pos.output_;
        }
    }
    // 速度模式: vel_target 由 API 直接设，跳过位置环

    // 速度环（两种模式共用 vel_target）
    axis.pid_vel.setSetpoint(axis.vel_target + axis.vel_ff);
    axis.pid_vel.setFeedforward(axis.curr_ff);
    axis.pid_vel.update(axis.actual_vel);
    axis.motor.setCurrent(axis.pid_vel.output_);
}

/*============================================
 *  定时器回调 — 1ms 周期，遍历所有实例
 ============================================*/

void Gimbal::update()
{
    // 读取 AHRS 纠偏后角速度 [rad/s] → [deg/s]
    float gyro_yaw   = ahrs_->output_.gyro_b[2] * RAD_TO_DEG;
    float gyro_pitch = ahrs_->output_.gyro_b[1] * RAD_TO_DEG;

    // 填入 yaw 轴
    yaw_.actual_vel   = gyro_yaw;
    yaw_.actual_angle  = ahrs_->output_.yaw_total;
    axisUpdate(yaw_);

    // 填入 pitch 轴
    pitch_.actual_vel   = gyro_pitch;
    pitch_.actual_angle = ahrs_->output_.euler[1];
    axisUpdate(pitch_);
}

void Gimbal::timCallback(void* device)
{
    (void)device;
    for (uint8_t i = 0; i < idx_; i++)
        instances_[i]->update();
}

/*============================================
 *  init
 ============================================*/

void Gimbal::init(const Config& config)
{
    if (!config.ahrs || idx_ >= GIMBAL_MAX)
        return;

    ahrs_      = config.ahrs;
    yaw_min_   = config.yaw_min;
    yaw_max_   = config.yaw_max;
    pitch_min_ = config.pitch_min;
    pitch_max_ = config.pitch_max;

    // --- 分频 ---
    yaw_.pos_freq_div   = config.pos_freq_div_yaw;
    pitch_.pos_freq_div = config.pos_freq_div_pitch;

    // --- 注册电机（内部 PID 清零）---
    DJIMotor::Config motor_cfg = {
        .can_handle    = config.can_handle,
        .motor_id      = config.motor_id_yaw,
        .motor_type    = DJIMotor_6020,
        .pid_angle     = { .kp = 0, .ki = 0, .kd = 0 },
        .pid_velocity  = { .kp = 0, .ki = 0, .kd = 0 },
        .pos_freq_div  = 1,
        .initial_angle = config.initial_angle_yaw,
    };
    yaw_.motor.init(motor_cfg);

    motor_cfg.motor_id      = config.motor_id_pitch;
    motor_cfg.initial_angle = config.initial_angle_pitch;
    pitch_.motor.init(motor_cfg);

    // --- 位置环 PID — 清除 feedforward ---
    yaw_.pid_pos.init(config.pid_yaw_pos);
    pitch_.pid_pos.init(config.pid_pitch_pos);
    yaw_.pid_pos.clearFeatures(PID::FeatureFeedforward);
    pitch_.pid_pos.clearFeatures(PID::FeatureFeedforward);

    // --- 速度环 PID — 启用 feedforward ---
    PID::Config vel_cfg = config.pid_yaw_vel;
    vel_cfg.feedforward_gain = 1.0f;   // 力矩前馈增益，默认1.0f
    yaw_.pid_vel.init(vel_cfg);
    yaw_.pid_vel.setFeatures(PID::FeatureFeedforward);

    vel_cfg = config.pid_pitch_vel;
    vel_cfg.feedforward_gain = 1.0f;
    pitch_.pid_vel.init(vel_cfg);
    pitch_.pid_vel.setFeatures(PID::FeatureFeedforward);

    // --- 注册到全局列表 ---
    instances_[idx_++] = this;

    // --- 定时器（首次注册时创建）---
    if (idx_ == 1) {
        tim_.setCallback(timCallback, nullptr);
        TIM::Config tim_cfg = { .htim = config.htim };
        tim_.init(tim_cfg);
    }
}

/*============================================
 *  公共 API
 ============================================*/

static float clamp_angle(float angle, float min, float max)
{
    if (min == 0.0f && max == 0.0f) return angle;
    if (angle < min) return min;
    if (angle > max) return max;
    return angle;
}

void Gimbal::enablePitch(uint8_t enable)
{
    pitch_.motor.setEnable(enable);
}

void Gimbal::enableYaw(uint8_t enable)
{
    yaw_.motor.setEnable(enable);
}

void Gimbal::enable(uint8_t enable)
{
    enablePitch(enable);
    enableYaw(enable);
}

void Gimbal::setTarget(float yaw, float pitch)
{
    yaw_.mode   = Mode::Position;
    pitch_.mode = Mode::Position;
    yaw_.target   = yaw;
    yaw_.target    = clamp_angle(yaw_.target,   yaw_min_,   yaw_max_);
    pitch_.target = pitch;
    pitch_.target  = clamp_angle(pitch_.target, pitch_min_, pitch_max_);
}

// 当前角度 + 增量
void Gimbal::setIncrement(float yaw_delta, float pitch_delta)
{
    yaw_.mode   = Mode::Position;
    pitch_.mode = Mode::Position;
    yaw_.target   = yaw_delta + ahrs_->output_.yaw_total;
    yaw_.target    = clamp_angle(yaw_.target,   yaw_min_,   yaw_max_);
    pitch_.target = pitch_delta + ahrs_->output_.euler[1];
    pitch_.target  = clamp_angle(pitch_.target, pitch_min_, pitch_max_);
}

void Gimbal::setPitchAbsolute(float angle)
{
    pitch_.target = clamp_angle(angle, pitch_min_, pitch_max_);
}

void Gimbal::setVelocityFF(float yaw_ff, float pitch_ff)
{
    yaw_.vel_ff   = yaw_ff;
    pitch_.vel_ff = pitch_ff;
}

void Gimbal::setCurrentFF(float yaw_ff, float pitch_ff)
{
    yaw_.curr_ff   = yaw_ff;
    pitch_.curr_ff = pitch_ff;
}

void Gimbal::setMode(Mode yaw_mode, Mode pitch_mode)
{
    yaw_.mode   = yaw_mode;
    pitch_.mode = pitch_mode;
}

void Gimbal::setVelocity(float yaw_vel, float pitch_vel)
{
    // // yaw 限幅：超出边界时禁止继续往外走
    // if (yaw_.actual_angle > yaw_max_ && yaw_max_ != 0.0f) {
    //     if (yaw_vel > 0.0f) yaw_vel = 0.0f;
    // } else if (yaw_.actual_angle < yaw_min_ && yaw_min_ != 0.0f) {
    //     if (yaw_vel < 0.0f) yaw_vel = 0.0f;
    // }

    // // pitch 限幅
    // if (pitch_.actual_angle > pitch_max_) {
    //     if (pitch_vel < 0.0f) pitch_vel = 0.0f;
    // } else if (pitch_.actual_angle < pitch_min_) {
    //     if (pitch_vel > 0.0f) pitch_vel = 0.0f;
    // }

    yaw_.mode       = Mode::Velocity;
    pitch_.mode     = Mode::Velocity;
    yaw_.vel_target   = yaw_vel;
    pitch_.vel_target = pitch_vel;
}

float Gimbal::getYawAngle()
{
    return yaw_.motor.angle_;
}
