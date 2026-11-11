/**
 * @file    dm_motor.h
 * @brief   达妙关节电机（位置 / 速度 / 电流 / MIT；电流 = MIT 纯 t_ff）
 * @note    壳子参考 DJIMotor（公开用户量 + private FeedbackRaw）；单位 rad、rad/s、N.m。
 *          本工程为 CAN（bsp_can），Config.can_handle 为 CAN_HandleTypeDef*。
 *          控制模式在 Config.mode 指定；运行期 requestMode() 由定时回调异步切换。
 *          setEnable(0) 已失能则不再发管理帧；失能态由所选定时器心跳持续发失能管理帧。
 *          setEnable(1) 发送使能管理帧后改发控制帧。电调因此在失能态仍持续回帧。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）
 */
#pragma once

#include "dm_motor_def.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "daemon.h"
#include "bsp_log_config.h"

class DMMotor {
public:
    /// 控制模式（对应 DJIMotor::PidMode；多 Mit）。顺序连续，可不写值。
    enum class Mode : uint8_t {
        Current = 0,
        Velocity,
        Position,
        Mit,
    };

    enum class ModeSwitchStatus : uint8_t {
        Ready = 0,
        Disabling,
        Writing,
        Waiting,
        Enabling,
    };

    /// D[0] 高 4 位；识别／读取异常按用户提供的协议图补齐，未定义值原样保留。
    enum class State : uint8_t {
        Disabled = 0x0,
        Running = 0x1,
        MotorUnrecognized = 0x2,
        OutputShaftUnrecognized = 0x3,
        OffsetReadError = 0x5,
        EncoderReadError = 0x7,
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
        uint8_t state;       ///< D[0] 高 4 位状态（原始）
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
        Mode mode = Mode::Current;        ///< 初始模式；init 写电调 CTRL_MODE
        uint8_t direction = DM_DIRECTION_NORMAL;  ///< DM_DIRECTION_NORMAL / REVERT
        float reduction_ratio = 1.0f;     ///< 额外传动比，电调输出轴 / 用户轴；不重复填写电机内置减速比
        float initial_angle = 0.0f;       ///< 初始角度 [rad]；内部 angle_offset_ = -initial_angle
        TIM_HandleTypeDef* htim;          ///< 本实例心跳定时器（建议 1ms）
        uint32_t daemon_cycle = 100;      ///< 失联超时 [ms]，Daemon 按定时器周期换成拍数
        float p_min = 0, p_max = 0;       ///< 0 表示驱动默认量程
        float v_min = 0, v_max = 0;
        float t_min = 0, t_max = 0;
    };

    // —— 实例变量 ——
    // 用户坐标 / 解码反馈
    float angle_ = 0.0f;            ///< [rad] 连续多圈用户角，方向与零偏变换后；须检查 angle_valid_
    float angle_raw_ = 0.0f;        ///< [rad] 电调协议角：减速后、Reverse/offset 前
    float velocity_ = 0.0f;         ///< [rad/s]
    float current_ = 0.0f;          ///< 扭矩 [N.m]（命名对齐 DJI current_）
    State state_ = State::Disabled; ///< D[0] 高 4 位：0 失能，1 工作中
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
    uint8_t angle_valid_ = 0;        ///< 多圈连续性有效；失联/置零后须显式重建，不能由通信恢复代替

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
    uint16_t ecd_last_ = 0;
    uint8_t angle_initialized_ = 0;
    Mode pending_mode_ = Mode::Current;
    Mode desired_mode_ = Mode::Current;
    ModeSwitchStatus mode_switch_status_ = ModeSwitchStatus::Ready;
    uint8_t mode_switch_wait_ticks_ = 0;
    uint8_t resume_after_mode_switch_ = 0;

    // —— static 变量 ——
    static DMMotor* instances_[DM_MOTOR_MAX_INSTANCE];
    static uint8_t idx_;

    // —— static 函数 ——
public:
#if ROBOT_RTT_PRINT && (ROBOT_RTT_REPORT_MOTOR_DETAIL || ROBOT_RTT_REPORT_CLEAR_ERROR)
    static const char* stateName(State state);
#endif
    static uint8_t isFaultState(State state)
    {
        return state == State::MotorUnrecognized || state == State::OutputShaftUnrecognized ||
               state == State::OffsetReadError || state == State::EncoderReadError ||
               static_cast<uint8_t>(state) >= 0x8u;
    }

private:
    static void decodeCallback(void* device);
    static void loseCallback(void* device);
    static void timCallback(void* device);
    static uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits);
    static float uintToFloat(uint16_t x, float x_min, float x_max, uint8_t bits);
    static DMMotor_EscCtrlMode modeToEsc(Mode mode);

    // —— 实例函数 ——
public:
    void init(const Config& config);

    void setEnable(uint8_t motor_enable);  ///< 1：使能；0：失能，心跳持续发失能帧
#if ROBOT_RTT_PRINT && ROBOT_RTT_REPORT_CLEAR_ERROR
    void clearError(const char* reason = nullptr, const char* who = nullptr);
#else
    void clearError();
#endif
    void setZero();
    /// 丢弃历史圈数，下一个匹配 ID 的反馈以协议角重建用户角；不发送电调置零命令。
    /// 置零后须等电调完成、旧反馈排空，再调用；不能恢复失联或掉电期间的圈数。
    void resetAngleTracking();
    uint8_t running() const { return state_ == State::Running ? 1u : 0u; }
    uint8_t faulted() const
    {
        return isFaultState(state_);
    }
    /// 保留原有自动清错范围；新增识别／读取异常仅诊断，不推定可以自动清除。
    uint8_t shouldAutoClear() const
    {
        return static_cast<uint8_t>(state_) >= 0x8u ? 1u : 0u;
    }

    /// 异步切换控制模式；切换中也记下目标，当前切完接着切，不丢。
    /// 完成前会重置为当前位置、零速度、零力矩；新目标应在 modeSwitching()==0 后设置。
    uint8_t requestMode(Mode mode);
    Mode mode() const;
    /// Ready 表示本地发送时序完成；当前驱动未解析电调的参数写入应答。
    ModeSwitchStatus modeSwitchStatus() const;
    uint8_t modeSwitching() const;

    void setKp(float kp);
    void setKd(float kd);

    void setAngle(float angle);                 ///< 电调位置坐标经方向/零偏变换后的目标；非无限多圈目标
    void setAngle(float angle, float velocity); ///< |velocity|：梯形匀速段上限 [rad/s]
    void setVelocity(float velocity);           ///< 需 Mode::Velocity
    void setCurrent(float current);             ///< 力矩 [N.m]（MIT 纯 t_ff）；需 Mode::Current
    void setMit(float angle, float velocity, float current, float kp, float kd);  ///< 需 Mode::Mit
    void setMit(float angle, float velocity, float current);  ///< 用已设 kp_ kd_

private:
    uint8_t sendCmd(DMMotor_Cmd cmd);
    uint8_t writeRegU32(uint8_t rid, uint32_t value);
    void update();  ///< TIM 周期入口（对齐 DJIMotor::update）
    void updateModeSwitch();
    void beginModeSwitch();
    void enterReady();
    void primeTargetsForModeSwitch();
    void packMit(uint8_t out[8], float p, float v, float kp, float kd, float t_ff) const;
};
