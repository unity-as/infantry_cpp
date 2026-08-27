#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "dji_motor_def.h"
#include "bsp_can.h"
#include "pid.h"
#include "daemon.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DJIM_MAX_INSTANCE 16
#define DJIM_MAX_GROUP 6

typedef enum
{
    FOC_CURRENT_LOOP_CONTROL = 0,
    FOC_VELOCITY_LOOP_CONTROL,
    FOC_POSITION_LOOP_CONTROL,
} DJIMotor_PID_Mode;

typedef struct
{
    uint16_t ecd;//电机编码器值
    int16_t rpm;//电机转速
    int16_t curr;//电机电流
    uint8_t temp;//电机温度
}DJIMotor_feedback_raw;

typedef struct
{
    CAN_Instance *djim_can;
    DJIMotor_feedback_raw feedback_raw;

    DJIMotor_Type motor_type;
    PID_Instance *pid_angle;
    PID_Instance *pid_velocity;
//友情提醒：  PID_Instance pid_current; 电调是内置了电流环的，不需要我们自己写。硬要GM6020调成电压模式写，1kHz的电流环还不如直接电压开环控制
    Daemon_Instance *daemon_lose; //失联守护实例

    float angle;          //电机角度，单位度
    float angle_offset;   //角度偏移量 [deg]（内部自动取反：offset = -initial_angle）
    float velocity;       //电机速度，单位度/s（已一阶低通滤波）
    float velocity_raw;   //电机速度原始值，单位度/s（未滤波）
    float current;       //电机电流，单位A       注：6020的电压模式时，
    float temperature;   //电机温度，单位摄氏度

    float target_angle;    //位置环的目标角度，单位度
    float target_velocity; //速度环的目标速度，单位度/s
    float target_current;  //发送给电调的目标电流，单位A

    float velocity_feedforward; //速度前馈，单位度/s
    float current_feedforward;    //力矩前馈，单位度A

    DJIMotor_PID_Mode pid_mode;  //PID控制模式，位置环、速度环还是电流环
    uint16_t ecd_last;           //上次的编码器值,用于做增量式位置
    uint8_t motor_enable;        //电机使能，关闭时不更新PID，直接输出0V
    uint8_t motor_valid;         //电机通信有效，收到首帧反馈后置1，失联守护触发后置0
    uint8_t pos_freq_div;          //位置环分频，位置环更新频率=1000Hz/pos_freq_div
    uint8_t div_cnt;             //分频计数器

    uint8_t motor_id;                             // 电机ID
    uint8_t direction;                            // 方向：DJIM_DIRECTION_NORMAL / REVERT
    float reduction_ratio;                        // 减速比，电机轴 / 输出轴
    uint8_t *command_ptr;//指向电机控制命令的指针，快速访问
}DJIMotor_Instance;

typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint8_t motor_id;                             // 电机ID

    DJIMotor_Type motor_type;                         // 电机型号
    uint8_t direction;                                // 方向：DJIM_DIRECTION_NORMAL / REVERT
    float reduction_ratio;                            // 减速比，电机轴 / 输出轴，默认 1.0
    PID_Init_Config_s pid_angle;                    //位置环PID配置
    PID_Init_Config_s pid_velocity;                 //速度环PID配置
    uint8_t pos_freq_div;          //位置环分频，位置环更新频率=1000Hz/pos_freq_div
    float initial_angle;           //电机初始角度 [deg]（正值=电机当前朝向），内部取反存入 offset
}DJIMotor_Init_Config_s;

DJIMotor_Instance *DJIMotor_Register(DJIMotor_Init_Config_s *config);                   //注册电机实例 config:电机初始化配置
void DJIMotor_Set_Enable(DJIMotor_Instance *instance, uint8_t motor_enable);            //设置电机使能 0:关闭电机，1：开启电机
void DJIMotor_Set_Angle(DJIMotor_Instance *instance, float angle);                      //设置电机角度（最短路径处理） 单位度
void DJIMotor_Set_Angle_Circular(DJIMotor_Instance *instance, float angle);             //设置电机角度（多圈角度处理） 单位度
void DJIMotor_Set_Angle_Increment(DJIMotor_Instance *instance, float angle_increment);  //设置电机角度增量 单位度
void DJIMotor_Set_Velocity(DJIMotor_Instance *instance, float velocity);                //设置电机速度 单位度/s
void DJIMotor_Set_Current(DJIMotor_Instance *instance, float current);                  //设置电机电流 单位A
void DJIMotor_Set_VelocityFF(DJIMotor_Instance *instance, float velocity);              //设置速度前馈 单位度/s
void DJIMotor_Set_CurrentFF(DJIMotor_Instance *instance, float current);                //设置力矩前馈 单位度A
void DJIMotor_TimbaseSelect(TIM_HandleTypeDef *htim);                                   //选择定时器作为PID更新的时间基准，建议使用1000Hz的定时器中断

// 获取带偏移的角度 [deg]（用户坐标系，已含方向+减速比+offset）
#define DJIM_GET_ANGLE(instance) ((instance)->angle)

// 获取电机电流 [A]（用户坐标系，已含方向）
#define DJIM_GET_CURRENT(instance) ((instance)->current)

#define DJIM_VELOCITY_LPF_ALPHA 0.15f   // 速度反馈一阶低通系数（0~1，越小滤波越强）

// 获取电机转速 [deg/s]（用户坐标系，已含方向+减速比，已滤波）
#define DJIM_GET_VELOCITY(instance) ((instance)->velocity)

// 获取电机转速原始值 [deg/s]（用户坐标系，未滤波）
#define DJIM_GET_VELOCITY_RAW(instance) ((instance)->velocity_raw)

#ifdef __cplusplus
}
#endif

#endif