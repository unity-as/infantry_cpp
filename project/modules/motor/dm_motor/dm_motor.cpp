/**
 * @file    dm_motor.cpp
 * @brief   达妙关节电机：解码 + 四模式组帧 + 管理命令 + TIM 心跳
 * @note    自 leg_main 移植；MIT/反馈手拼跨字节字段；0x100/0x200 为 float 小端。
 *          每实例 Config.htim 驱动 update；Daemon 共用同一时基。经典 CAN。
 */
#include "dm_motor.h"

#include "bsp_dwt.h"
#include <string.h>

DMMotor* DMMotor::instances_[DM_MOTOR_MAX_INSTANCE] = {};
uint8_t DMMotor::idx_ = 0;

namespace {

float clamp(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

void writeFloatLE(uint8_t* dst, float v)
{
    memcpy(dst, &v, sizeof(float));
}

}  // namespace

void DMMotor::decodeCallback(void* device)
{
    DMMotor* instance = static_cast<DMMotor*>(device);
    instance->decode(instance->dm_can_.rx_buff_);
}

void DMMotor::loseCallback(void* device)
{
    DMMotor* instance = static_cast<DMMotor*>(device);
    instance->motor_valid_ = 0;
}

void DMMotor::timCallback(void* device)
{
    DMMotor* instance = static_cast<DMMotor*>(device);
    instance->update();
}

void DMMotor::init(const Config& config)
{
    if (idx_ >= DM_MOTOR_MAX_INSTANCE)
        return;

    control_id_ = config.control_id;
    feedback_id_ = config.feedback_id;
    direction_ = config.direction;
    reduction_ratio_ = config.reduction_ratio > 0.0f ? config.reduction_ratio : 1.0f;
    angle_offset_ = -config.initial_angle;

    p_min_ = DM_P_MIN;
    p_max_ = DM_P_MAX;
    v_min_ = DM_V_MIN;
    v_max_ = DM_V_MAX;
    t_min_ = DM_T_MIN;
    t_max_ = DM_T_MAX;
    if (config.p_min != 0.0f || config.p_max != 0.0f) {
        p_min_ = config.p_min;
        p_max_ = config.p_max;
    }
    if (config.v_min != 0.0f || config.v_max != 0.0f) {
        v_min_ = config.v_min;
        v_max_ = config.v_max;
    }
    if (config.t_min != 0.0f || config.t_max != 0.0f) {
        t_min_ = config.t_min;
        t_max_ = config.t_max;
    }

    CAN::Config can_config = {
        .can_handle = config.can_handle,
        .rx_id = config.feedback_id,
    };
    dm_can_.setCallback(decodeCallback, this);
    dm_can_.init(can_config);

    Daemon::Config daemon_config = {
        .tim_config = { .htim = config.htim },
        .cycle = config.daemon_cycle == 0 ? 100u : config.daemon_cycle,
        .daemon_callback = loseCallback,
        .device = this,
    };
    daemon_lose_.init(daemon_config);

    tim_.setCallback(timCallback, this);
    TIM::Config tim_config = { .htim = config.htim };
    tim_.init(tim_config);

    instances_[idx_++] = this;
}

uint16_t DMMotor::floatToUint(float x, float x_min, float x_max, uint8_t bits)
{
    x = clamp(x, x_min, x_max);
    const float span = x_max - x_min;
    if (span <= 0.0f)
        return 0;
    return static_cast<uint16_t>((x - x_min) * static_cast<float>((1u << bits) - 1u) / span);
}

float DMMotor::uintToFloat(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    const float span = x_max - x_min;
    if (span <= 0.0f)
        return x_min;
    return static_cast<float>(x) * span / static_cast<float>((1u << bits) - 1u) + x_min;
}

void DMMotor::packMit(uint8_t out[8], float p, float v, float kp, float kd, float t_ff) const
{
    if (direction_ == DM_DIRECTION_REVERT) {
        p = -p;
        v = -v;
        t_ff = -t_ff;
    }
    p *= reduction_ratio_;
    v *= reduction_ratio_;

    const uint16_t p_u = floatToUint(p, p_min_, p_max_, 16);
    const uint16_t v_u = floatToUint(v, v_min_, v_max_, 12);
    const uint16_t kp_u = floatToUint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    const uint16_t kd_u = floatToUint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    const uint16_t t_u = floatToUint(t_ff, t_min_, t_max_, 12);

    out[0] = static_cast<uint8_t>(p_u >> 8);
    out[1] = static_cast<uint8_t>(p_u);
    out[2] = static_cast<uint8_t>(v_u >> 4);
    out[3] = static_cast<uint8_t>(((v_u & 0x0Fu) << 4) | (kp_u >> 8));
    out[4] = static_cast<uint8_t>(kp_u);
    out[5] = static_cast<uint8_t>(kd_u >> 4);
    out[6] = static_cast<uint8_t>(((kd_u & 0x0Fu) << 4) | (t_u >> 8));
    out[7] = static_cast<uint8_t>(t_u);
}

void DMMotor::decode(const uint8_t rx[8])
{
    const uint8_t id = static_cast<uint8_t>(rx[0] & 0x0Fu);
    const uint8_t err = static_cast<uint8_t>(rx[0] >> 4);
    if (id != static_cast<uint8_t>(control_id_ & 0x0Fu))
        return;

    feedback_raw_.id = id;
    feedback_raw_.err = err;
    feedback_raw_.pos = static_cast<uint16_t>((rx[1] << 8) | rx[2]);
    feedback_raw_.vel = static_cast<uint16_t>((rx[3] << 4) | (rx[4] >> 4));
    feedback_raw_.torque = static_cast<uint16_t>(((rx[4] & 0x0Fu) << 8) | rx[5]);
    feedback_raw_.mos_temp = rx[6];
    feedback_raw_.rotor_temp = rx[7];

    float angle = uintToFloat(feedback_raw_.pos, p_min_, p_max_, 16);
    float velocity = uintToFloat(feedback_raw_.vel, v_min_, v_max_, 12);
    float current = uintToFloat(feedback_raw_.torque, t_min_, t_max_, 12);

    if (reduction_ratio_ != 0.0f) {
        angle /= reduction_ratio_;
        velocity /= reduction_ratio_;
    }
    if (direction_ == DM_DIRECTION_REVERT) {
        angle = -angle;
        velocity = -velocity;
        current = -current;
    }

    angle_ = angle + angle_offset_;
    velocity_ = velocity;
    current_ = current;
    err_ = static_cast<Err>(feedback_raw_.err);  // 未知码原样保留数值
    mos_temp_ = feedback_raw_.mos_temp;
    rotor_temp_ = feedback_raw_.rotor_temp;
    motor_valid_ = 1;
    daemon_lose_.reset();
}

void DMMotor::sendCmd(DMMotor_Cmd cmd)
{
    dm_can_.setTxId(control_id_ + DM_CTRL_ID_MIT);
    memset(dm_can_.tx_buff_, 0xFF, 7);
    dm_can_.tx_buff_[7] = static_cast<uint8_t>(cmd);
    dm_can_.transmit(1.0f);
}

void DMMotor::writeRegU32(uint8_t rid, uint32_t value)
{
    // 参数帧：StdId=0x7FF，{id_lo,id_hi,op=0x55,RID,u32 LE}
    dm_can_.setTxId(DM_PARAM_CAN_ID);
    dm_can_.tx_buff_[0] = static_cast<uint8_t>(control_id_ & 0xFFu);
    dm_can_.tx_buff_[1] = static_cast<uint8_t>((control_id_ >> 8) & 0xFFu);
    dm_can_.tx_buff_[2] = static_cast<uint8_t>(DM_PARAM_WRITE);
    dm_can_.tx_buff_[3] = rid;
    memcpy(&dm_can_.tx_buff_[4], &value, sizeof(uint32_t));
    dm_can_.transmit(1.0f);
}

void DMMotor::ensureEscMode(DMMotor_EscCtrlMode esc_mode)
{
    const uint8_t want = static_cast<uint8_t>(esc_mode);
    if (esc_mode_ == want)
        return;
    writeRegU32(DM_REG_CTRL_MODE, static_cast<uint32_t>(want));
    esc_mode_ = want;
    DWT_Delay_ms(2.0f);  // 给电调清内部指令一点时间
}

void DMMotor::setEscCtrlMode(DMMotor_EscCtrlMode esc_mode)
{
    writeRegU32(DM_REG_CTRL_MODE, static_cast<uint32_t>(esc_mode));
    esc_mode_ = static_cast<uint8_t>(esc_mode);
    DWT_Delay_ms(2.0f);
}

void DMMotor::update()
{
    if (!motor_enable_) {
        dm_can_.setTxId(control_id_ + DM_CTRL_ID_MIT);
        packMit(dm_can_.tx_buff_, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        dm_can_.transmit(1.0f);
        return;
    }

    // 用户坐标系 → 电机坐标系（与 angle_ = raw + angle_offset_ 互逆）
    const float angle_motor = target_angle_ - angle_offset_;

    switch (mode_) {
    case Mode::Position: {
        float p = angle_motor;
        float v = target_velocity_;
        if (direction_ == DM_DIRECTION_REVERT) {
            p = -p;
            v = -v;
        }
        p *= reduction_ratio_;
        v *= reduction_ratio_;
        dm_can_.setTxId(control_id_ + DM_CTRL_ID_POS_VEL);
        writeFloatLE(&dm_can_.tx_buff_[0], p);
        writeFloatLE(&dm_can_.tx_buff_[4], v);
        break;
    }
    case Mode::Velocity: {
        float v = target_velocity_;
        if (direction_ == DM_DIRECTION_REVERT)
            v = -v;
        v *= reduction_ratio_;
        dm_can_.setTxId(control_id_ + DM_CTRL_ID_VEL);
        writeFloatLE(&dm_can_.tx_buff_[0], v);
        memset(&dm_can_.tx_buff_[4], 0, 4);
        break;
    }
    case Mode::Current:
        dm_can_.setTxId(control_id_ + DM_CTRL_ID_MIT);
        packMit(dm_can_.tx_buff_, 0.0f, 0.0f, 0.0f, 0.0f, target_current_);
        break;
    case Mode::Mit:
        dm_can_.setTxId(control_id_ + DM_CTRL_ID_MIT);
        packMit(dm_can_.tx_buff_, angle_motor, target_velocity_, kp_, kd_, target_current_);
        break;
    }

    dm_can_.transmit(1.0f);
}

void DMMotor::setEnable(uint8_t motor_enable)
{
    motor_enable_ = motor_enable ? 1u : 0u;
    if (motor_enable_)
        sendCmd(DM_CMD_ENABLE);
    else
        update();
}

void DMMotor::clearError()
{
    sendCmd(DM_CMD_CLEAR_ERROR);
}

void DMMotor::setZero()
{
    sendCmd(DM_CMD_SET_ZERO);
}

void DMMotor::setKp(float kp)
{
    kp_ = kp;
}

void DMMotor::setKd(float kd)
{
    kd_ = kd;
}

void DMMotor::setAngle(float angle)
{
    setAngle(angle, DM_VEL_UNLIMITED);
}

void DMMotor::setAngle(float angle, float velocity)
{
    ensureEscMode(DM_ESC_POS_VEL);
    mode_ = Mode::Position;
    target_angle_ = angle;
    target_velocity_ = velocity;
}

void DMMotor::setVelocity(float velocity)
{
    ensureEscMode(DM_ESC_VEL);
    mode_ = Mode::Velocity;
    target_velocity_ = velocity;
}

void DMMotor::setCurrent(float current)
{
    ensureEscMode(DM_ESC_MIT);
    mode_ = Mode::Current;
    target_current_ = current;
}

void DMMotor::setMit(float angle, float velocity, float current, float kp, float kd)
{
    kp_ = kp;
    kd_ = kd;
    setMit(angle, velocity, current);
}

void DMMotor::setMit(float angle, float velocity, float current)
{
    ensureEscMode(DM_ESC_MIT);
    mode_ = Mode::Mit;
    target_angle_ = angle;
    target_velocity_ = velocity;
    target_current_ = current;
}
