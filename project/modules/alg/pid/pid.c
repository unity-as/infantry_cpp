/**
 * @file pid.c
 * @author le
 * @brief 通用PID控制器模块，看起来乱乱的，可读性不咋地，功能没问题，运行起来比看起来整洁。不折磨编译器折磨自己吧。
 */

#include "pid.h"
#include <stdlib.h>
#include <string.h>

//PID 初始化
PID_Instance * PID_Init(PID_Init_Config_s *config)
{
    PID_Instance *instance = (PID_Instance *)malloc(sizeof(PID_Instance));
    memset(instance, 0, sizeof(PID_Instance));

    // 参数初始化
    instance->kp = config->kp;
    instance->ki = config->ki;
    instance->kd = config->kd;
    instance->mode = config->mode;
    instance->features=config->features;
    instance->period = config->period == 0 ? 1.0f : config->period;

    instance->integral_limit = config->integral_limit;
    instance->derivative_limit = config->derivative_limit;
    instance->dead_zone = config->dead_zone;
    instance->output_min = config->output_min;
    instance->output_max = config->output_max;
    instance->feedforward_gain = config->feedforward_gain == 0 ? 1.0f : config->feedforward_gain;

    instance->filter_alpha = config->filter_alpha;
    if(instance->filter_alpha <= 0.0f) instance->filter_alpha = 0.3f;
    else if(instance->filter_alpha > 1.0f) instance->filter_alpha = 1.0f;
    instance->kp_extra = config->kp_extra;
    instance->kd_extra = config->kd_extra;
    return instance;
}

//PID 重置
void PID_Reset(PID_Instance *instance)
{
    if(instance==NULL) return;
    instance->integral = 0.0f;
    instance->last_error = 0.0f;
    instance->last_feedback = 0.0f;
    instance->last_2_error = 0.0f;
    instance->last_2_feedback = 0.0f;
    instance->output = 0.0f;
}

//PID 设置积分项（反算抗饱和等场景需要直接读写积分累加器）
void PID_Set_Integral(PID_Instance *instance, float value)
{
    if(instance==NULL) return;
    instance->integral = value;
}

//PID 重置积分
void PID_Reset_Integral(PID_Instance *instance)
{
    PID_Set_Integral(instance, 0.0f);
}

//PID 参数设置
void PID_Set_Parameters(PID_Instance *instance, float kp, float ki, float kd)
{
    if(instance==NULL) return;
    instance->kp = kp;
    instance->ki = ki;
    instance->kd = kd;
}

//PID 设置目标值
void PID_Set_Setpoint(PID_Instance *instance, float setpoint)
{
    if(instance==NULL) return;
    instance->setpoint = setpoint;
}

//PID 设置前馈
void PID_Set_Feedforward(PID_Instance *instance, float feedforward)
{
    if(instance==NULL) return;
    instance->feedforward = feedforward;
}
//PID 额外功能是否开启
PID_FEATURE PID_Get_Features(PID_Instance *instance, PID_FEATURE feature)
{
    if(instance==NULL) return 0;
    return instance->features & feature;
}
//PID 功能开关
void PID_Set_Features(PID_Instance *instance, PID_FEATURE feature)
{
    if(instance==NULL) return;
    instance->features |= feature;
}
void PID_Clear_Features(PID_Instance *instance, PID_FEATURE feature)
{
    if(instance==NULL) return;
    instance->features &= ~feature;
}

//PID 更新
void PID_Update(PID_Instance *instance, float feedback)
{
    if(instance==NULL) return;

    // 一阶低通滤波输入
    if(instance->features & PID_FEATURE_FILTER)
        feedback = instance->filter_alpha * feedback + (1.0f - instance->filter_alpha) * instance->last_feedback;
    float error = instance->setpoint - feedback;

    // 死区处理
    if(instance->features & PID_FEATURE_DEAD_ZONE )
        if(error > -instance->dead_zone && error < instance->dead_zone)
            error = 0.0f;
    //变增益处理
    float kp = instance->kp;
    float kd = instance->kd;
    if(instance->features & PID_FEATURE_VARIABLE_GAIN)//线性增益
    {
        kp = instance->kp + (error > 0 ? error : -error)* instance->kp_extra;
        kd = instance->kd + (error > 0 ? error : -error)* instance->kd_extra;
    }

    if(instance->mode == PID_MODE_DELTA)//增量式
    {
        instance->integral = error * instance->period;
        //积分限幅 no
        //微分先行
        float derivative;
        if(instance->features & PID_FEATURE_DERIVATIVE_ON_MEASUREMENT)
            derivative = -(feedback - 2*instance->last_feedback + instance->last_2_feedback) / instance->period;
        else
            derivative = (error - 2*instance->last_error + instance->last_2_error) / instance->period;

        //微分限幅
        if(instance->features & PID_FEATURE_DERIVATIVE_LIMIT)
            if(derivative > instance->derivative_limit) derivative = instance->derivative_limit;
            else if(derivative < -instance->derivative_limit) derivative = -instance->derivative_limit;

        //输出计算
        instance->output += kp * (error - instance->last_error) + instance->ki * instance->integral + kd * derivative;

        //前馈处理
        if(instance->features & PID_FEATURE_FEEDFORWARD)
            instance->output += (instance->feedforward - instance->last_feedforward) * instance->feedforward_gain;

        instance->last_2_error = instance->last_error;
        instance->last_2_feedback = instance->last_feedback;
        instance->last_feedforward = instance->feedforward;
    }
    else//位置式
    {
        if(!(instance->features & PID_FEATURE_OUTPUT_LIMIT &&
            ((instance->output >= instance->output_max && error > 0) ||
            (instance->output <= instance->output_min && error < 0))))//抗积分饱和
            instance->integral += error * instance->period;
        //积分限幅
        if(instance->features & PID_FEATURE_INTEGRAL_LIMIT)
            if(instance->integral > instance->integral_limit) instance->integral = instance->integral_limit;
            else if(instance->integral < -instance->integral_limit) instance->integral = -instance->integral_limit;

        //微分先行
        float derivative;
        if(instance->features & PID_FEATURE_DERIVATIVE_ON_MEASUREMENT)
            derivative = -(feedback - instance->last_feedback) / instance->period; 
        else
            derivative = (error - instance->last_error) / instance->period; 

        //微分限幅
        if(instance->features & PID_FEATURE_DERIVATIVE_LIMIT)
            if(derivative > instance->derivative_limit) derivative = instance->derivative_limit;
            else if(derivative < -instance->derivative_limit) derivative = -instance->derivative_limit;

        //输出计算
        instance->output = kp * error + instance->ki * instance->integral + kd * derivative;

        //前馈处理
        if(instance->features & PID_FEATURE_FEEDFORWARD)
            instance->output += instance->feedforward * instance->feedforward_gain;
    }

    //输出限幅
    if(instance->features & PID_FEATURE_OUTPUT_LIMIT)
        if(instance->output > instance->output_max) instance->output = instance->output_max;
        else if(instance->output < instance->output_min) instance->output = instance->output_min;

    instance->last_error = error;
    instance->last_feedback = feedback;
}