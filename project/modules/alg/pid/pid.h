#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef enum
{
    PID_MODE_POSITION, // 位置式PID
    PID_MODE_DELTA     // 增量式PID
} PID_MODE;

typedef enum
{
    PID_FEATURE_NONE           = 0b00000000u,           // 无特殊功能
    PID_FEATURE_INTEGRAL_LIMIT = 0b00000001u,           // 积分限幅    (增量式不需要)
    PID_FEATURE_DERIVATIVE_LIMIT = 0b00000010u,         // 微分限幅
    PID_FEATURE_OUTPUT_LIMIT   = 0b00000100u,           // 输出限幅
    PID_FEATURE_DEAD_ZONE      = 0b00001000u,           // 死区处理
    PID_FEATURE_FILTER         = 0b00010000u,           // 输入滤波
    PID_FEATURE_VARIABLE_GAIN  = 0b00100000u,           // 变增益PID
    PID_FEATURE_DERIVATIVE_ON_MEASUREMENT = 0b01000000u,// 微分先行
    PID_FEATURE_FEEDFORWARD =0b10000000u,          // 前馈控制
} PID_FEATURE; 

typedef struct
{
    //基础参数
    float kp;      // 比例系数
    float ki;      // 积分系数
    float kd;      // 微分系数

    //高级参数
    float integral_limit;        // 积分限幅值
    float derivative_limit;      // 微分限幅值
    float output_min;            // 输出最小值
    float output_max;            // 输出最大值
    float dead_zone;             // 死区范围
    float filter_alpha;          // 滤波系数 (0~1)，越小滤波越强
    float feedforward_gain; // 前馈增益

    // 运行参数
    float setpoint; // 目标值
    float feedforward; // 前馈值
    float output; // PID的输出值

    //内部参数
    float integral; // PID的积分项                    （位置式PID）
    float last_feedback; // 上一次的反馈值          （微分先行）
    float last_2_feedback; // 上上次的反馈值           （增量式PID）
    float last_error; // 上一次的误差值
    float last_2_error; // 上上次的误差值            （增量式PID）
    float last_feedforward; // 上一次的前馈值          （增量式PID）

    // 变增益参数
    float kp_extra;        // 比例系数
    float kd_extra;        // 微分系数

    PID_MODE mode;   // PID模式
    PID_FEATURE features; // 功能设置
    uint8_t period; // 采样周期 ms
} PID_Instance;

typedef struct
{
    float kp;      // 比例系数
    float ki;      // 积分系数
    float kd;      // 微分系数

    PID_MODE mode;   // PID模式
    PID_FEATURE features;

    float integral_limit;        // 积分限幅值
    float derivative_limit;      // 微分限幅值
    float output_min;            // 输出最小值
    float output_max;            // 输出最大值
    float dead_zone;             // 死区范围
    float feedforward_gain; // 前馈增益
    float filter_alpha;          // 滤波系数 (0~1)，越小滤波越强

    // 变增益参数
    float kp_extra;        // 比例系数
    float kd_extra;        // 微分系数

    uint8_t period; // 采样周期 ms
} PID_Init_Config_s;

PID_Instance * PID_Init(PID_Init_Config_s *config);// 初始化PID

void PID_Set_Parameters(PID_Instance *instance, float kp, float ki, float kd);// 设置PID参数

PID_FEATURE PID_Get_Features(PID_Instance *instance, PID_FEATURE features);// 设置功能
void PID_Set_Features(PID_Instance *instance, PID_FEATURE features);// 设置功能
void PID_Clear_Features(PID_Instance *instance, PID_FEATURE features);// 清除功能

void PID_Set_Feedforward(PID_Instance *instance, float feedforward); // 设置前馈值
void PID_Set_Setpoint(PID_Instance *instance, float setpoint);// 设置目标值

void PID_Update(PID_Instance *instance, float feedback);// 更新PID

void PID_Set_Integral(PID_Instance *instance, float value);// 设置积分项
void PID_Reset_Integral(PID_Instance *instance);// 重置积分项（等价 Set_Integral(instance, 0)）
void PID_Reset(PID_Instance *instance);// 复位PID

#endif