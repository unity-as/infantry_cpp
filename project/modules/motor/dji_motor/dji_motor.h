/**
 * @file    dji_motor.h
 * @brief   DJI 电机（C → C++）
 * @note    从 C 版 dji_motor 迁移：struct DJIMotor_Instance → class DJIMotor，
 *          DJIMotor_Register → init、DJIMotor_Set_* → set*，逻辑不变，禁堆。
 *          原 DJIM_GET_* 宏删除，改为直接读公开字段（motor.angle_ 等）。
 *          成员顺序：嵌套类型 → 实例变量 → static 变量 → static 函数 → 实例函数
 *          （各组内 public → private）；布局对齐 leg_main（句柄仍为经典 CAN）。
 */
#pragma once

#include "dji_motor_def.h"
#include "bsp_can.h"
#include "pid.h"
#include "daemon.h"

#define DJIM_MAX_INSTANCE 16   // 最大电机实例数
#define DJIM_MAX_GROUP 6       // 最大发送组数

#define DJIM_VELOCITY_LPF_ALPHA 0.15f   // 速度反馈一阶低通系数（0~1，越小滤波越强）

class DJIMotor {
public:
    /// PID 控制模式
    enum class PidMode : uint8_t {
        Current = 0,    // FOC_CURRENT_LOOP_CONTROL
        Velocity,       // FOC_VELOCITY_LOOP_CONTROL
        Position,       // FOC_POSITION_LOOP_CONTROL
    };

    /// 反馈原始数据
    struct FeedbackRaw {
        uint16_t ecd;   // 电机编码器值
        int16_t rpm;    // 电机转速
        int16_t curr;   // 电机电流
        uint8_t temp;   // 电机温度
    };

    /// 初始化配置
    struct Config {
        CAN_HandleTypeDef* can_handle;  // CAN 句柄（&hcan1/2）
        uint8_t motor_id;               // 电机 ID（1~8）
        DJIMotor_Type motor_type;       // 电机型号
        uint8_t direction;              // 方向：DJIM_DIRECTION_NORMAL / REVERT
        float reduction_ratio;          // 减速比，电机轴 / 输出轴，默认 1.0
        PID::Config pid_angle;          // 位置环 PID 配置
        PID::Config pid_velocity;       // 速度环 PID 配置
        uint8_t pos_freq_div;           // 位置环分频，更新频率=1000Hz/pos_freq_div
        float initial_angle;            // 初始角度 [deg]（正值=电机当前朝向），内部取反存入 offset
    };

    // —— 实例变量 ——
    // 用户坐标 / 解码反馈
    float angle_ = 0.0f;              // 电机角度 [deg]
    float velocity_ = 0.0f;           // 电机速度 [deg/s]（已一阶低通滤波）
    float velocity_raw_ = 0.0f;       // 电机速度原始值 [deg/s]（未滤波）
    float current_ = 0.0f;            // 电机电流 [A]
    float temperature_ = 0.0f;        // 电机温度 [℃]
    // 指令 / 前馈
    float target_angle_ = 0.0f;       // 位置环目标角度 [deg]
    float target_velocity_ = 0.0f;    // 速度环目标速度 [deg/s]
    float target_current_ = 0.0f;     // 发送给电调的目标电流 [A]
    float velocity_feedforward_ = 0.0f;  // 速度前馈 [deg/s]
    float current_feedforward_ = 0.0f;   // 力矩前馈 [A]
    // 运行状态
    uint8_t motor_enable_ = 0;        // 电机使能，关闭时不更新 PID，直接输出 0
    uint8_t motor_valid_ = 0;         // 电机通信有效，收到首帧反馈后置 1，失联后置 0

private:
    // 子系统
    CAN djim_can_;
    PID pid_angle_;
    PID pid_velocity_;
    Daemon daemon_lose_;

    // 协议 raw（编码器帧）
    FeedbackRaw feedback_raw_ = {};

    // 身份 / 标定
    DJIMotor_Type motor_type_;
    uint8_t motor_id_;
    uint8_t direction_;
    float reduction_ratio_ = 1.0f;
    float angle_offset_ = 0.0f;                 // [deg]（offset = -initial_angle）

    // 控制内部
    PidMode pid_mode_ = PidMode::Current;
    uint16_t ecd_last_ = 0;
    uint8_t pos_freq_div_ = 0;
    uint8_t div_cnt_ = 0;
    uint8_t* command_ptr_ = nullptr;

    // —— static 变量 ——
    static DJIMotor* instances_[DJIM_MAX_INSTANCE];   // 电机实例注册表
    static uint8_t idx_;                              // 已注册电机数
    static CAN tx_group_pool_[DJIM_MAX_GROUP];        // 发送组 CAN 实例池
    static uint8_t group_idx_;                        // 已注册发送组数
    static TIM tim_;                                  // PID 更新定时器
    static TIM_HandleTypeDef* timbase_htim_;          // timbaseSelect 选定的时基（供后注册电机补绑 daemon）

    // —— static 函数 ——
public:
    static void timbaseSelect(TIM_HandleTypeDef* htim);  // 替代 DJIMotor_TimbaseSelect

private:
    static void decodeCallback(void* device);   // 替代 DecodeDJIMotor
    static void loseCallback(void* device);     // 替代 DJIMotor_Lose
    static void timCallback(void* device);      // 替代 DJIMotor_TimHandler
    static CAN* getGroup(DJIMotor* instance);   // 替代 DJIMotor_Get_Group

    // —— 实例函数 ——
public:
    void init(const Config& config);                 // 替代 DJIMotor_Register
    void setEnable(uint8_t motor_enable);            // 替代 DJIMotor_Set_Enable
    void setAngle(float angle);                      // 替代 DJIMotor_Set_Angle（最短路径）
    void setAngleCircular(float angle);              // 替代 DJIMotor_Set_Angle_Circular（多圈）
    void setAngleIncrement(float angle_increment);   // 替代 DJIMotor_Set_Angle_Increment
    void setVelocity(float velocity);                // 替代 DJIMotor_Set_Velocity
    void setCurrent(float current);                  // 替代 DJIMotor_Set_Current
    void setVelocityFF(float velocity);              // 替代 DJIMotor_Set_VelocityFF
    void setCurrentFF(float current);                // 替代 DJIMotor_Set_CurrentFF

private:
    void currentCommand();                      // 替代 DJIM_Current_Command
    void update();                              // 替代 DJIMotor_Update
};
