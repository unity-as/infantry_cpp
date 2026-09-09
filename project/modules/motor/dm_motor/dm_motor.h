/**
 * @file    dm_motor.h
 * @brief   达妙关节电机（位置 / 速度 / 电流 / MIT；电流 = MIT 纯 t_ff）
 * @note    自 leg_main 移植；壳子参考 DJIMotor（公开用户量 + private FeedbackRaw）；单位 rad、rad/s、N.m。
 *          本工程为经典 CAN（bsp_can），Config.can_handle 为 CAN_HandleTypeDef*。
 *          setEnable(0) 软失能并发 0 电流；setEnable(1) 软使能并发 0xFC。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）
 */
#pragma once

#include "dm_motor_def.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "daemon.h"

class DMMotor {
public:
    /// 控制模式（对应 DJIMotor::PidMode；多 Mit）。顺序连续，可不写值。
    enum class Mode : uint8_t {
        Current = 0,
        Velocity,
        Position,
        Mit,
    };

    /// 反馈故障码 = D[0] 高 4 位。协议取值不连续，必须与手册半字节一致。
    enum class Err : uint8_t {
        None = 0x0,
        OverVoltage = 0x8,
        UnderVoltage = 0x9,
        OverCurrent = 0xA,
        MosOverTemp = 0xB,
        CoilOverTemp = 0xC,
        LostComm = 0xD,
        Overload = 0xE,
    };

    struct FeedbackRaw {
        uint8_t id;          ///< D[0] 低 4 位（与 control_id 低 4 位核验）
        uint8_t err;         ///< D[0] 高 4 位故障码（原始）
        uint16_t pos;
        uint16_t vel;
        uint16_t torque;
        uint8_t mos_temp;
        uint8_t rotor_temp;
    };

    /// Config 按达妙协议，不强制与 DJI 字段一一对应
    struct Config {
        CAN_HandleTypeDef* can_handle;
        uint32_t control_id;              ///< 控制帧基 ID（助手 CANID）
        uint32_t feedback_id;             ///< 反馈帧 ID（助手 MasterID）
        uint8_t direction = DM_DIRECTION_NORMAL;  ///< DM_DIRECTION_NORMAL / REVERT
        float reduction_ratio = 1.0f;     ///< 减速比，电机轴 / 输出轴，默认 1.0
        float initial_angle = 0.0f;       ///< 初始角度 [rad]；内部 angle_offset_ = -initial_angle
        TIM_HandleTypeDef* htim;          ///< 本实例心跳定时器（建议 1ms）
        uint32_t daemon_cycle = 100;      ///< 失联超时（定时器周期数）
        float p_min = 0, p_max = 0;       ///< 0 表示驱动默认量程
        float v_min = 0, v_max = 0;
        float t_min = 0, t_max = 0;
    };

    // —— 实例变量 ——
    // 用户坐标 / 解码反馈
    float angle_ = 0.0f;            ///< [rad]
    float velocity_ = 0.0f;         ///< [rad/s]
    float current_ = 0.0f;          ///< 扭矩 [N.m]（命名对齐 DJI current_）
    Err err_ = Err::None;           ///< 反馈故障码
    uint8_t mos_temp_ = 0;          ///< MOS 温度 [℃]
    uint8_t rotor_temp_ = 0;        ///< 线圈温度 [℃]
    // 指令 / MIT 增益
    float target_angle_ = 0.0f;     ///< 位置 / MIT 位置给定 [rad]
    float target_velocity_ = 0.0f;  ///< [rad/s]
    float target_current_ = 0.0f;   ///< 力矩给定 [N.m]
    float kp_ = 0.0f;
    float kd_ = 0.0f;
    // 运行状态
    uint8_t motor_enable_ = 0;
    uint8_t motor_valid_ = 0;

private:
    // 子系统
    CAN dm_can_;
    TIM tim_;                       ///< 本实例控制心跳（建议 1ms）
    Daemon daemon_lose_;

    // 协议 raw（反馈帧半字节/量化值）
    FeedbackRaw feedback_raw_{};

    // 身份 / 标定 / 量程
    uint32_t control_id_ = 0;
    uint32_t feedback_id_ = 0;
    uint8_t direction_ = DM_DIRECTION_NORMAL;
    float reduction_ratio_ = 1.0f;
    float angle_offset_ = 0.0f;  ///< = -initial_angle
    float p_min_ = DM_P_MIN, p_max_ = DM_P_MAX;
    float v_min_ = DM_V_MIN, v_max_ = DM_V_MAX;
    float t_min_ = DM_T_MIN, t_max_ = DM_T_MAX;

    // 控制内部
    Mode mode_ = Mode::Current;

    // —— static 变量 ——
    static DMMotor* instances_[DM_MOTOR_MAX_INSTANCE];
    static uint8_t idx_;

    // —— static 函数 ——
    static void decodeCallback(void* device);
    static void loseCallback(void* device);
    static void timCallback(void* device);
    static uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits);
    static float uintToFloat(uint16_t x, float x_min, float x_max, uint8_t bits);

    // —— 实例函数 ——
public:
    void init(const Config& config);

    void setEnable(uint8_t motor_enable);  ///< 1：使能+0xFC；0：软失能并立即发 0 电流
    void clearError();
    void setZero();

    void setKp(float kp);
    void setKd(float kd);

    void setAngle(float angle);                 ///< 差分速度默认 0
    void setAngle(float angle, float velocity); ///< velocity：梯形匀速段上限 [rad/s]
    void setVelocity(float velocity);
    void setCurrent(float current);             ///< 力矩给定 [N.m]（MIT 纯 t_ff）
    void setMit(float angle, float current);    ///< 差分速度默认 0
    void setMit(float angle, float velocity, float current);

private:
    void sendCmd(DMMotor_Cmd cmd);
    void update();  ///< TIM 周期入口（对齐 DJIMotor::update）
    void packMit(uint8_t out[8], float p, float v, float kp, float kd, float t_ff) const;
    void decode(const uint8_t rx[8]);
};
