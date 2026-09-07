/**
 * @file    example_chassis.c
 * @brief   RTOS 应用层黄金例程（精简提炼版）
 *
 * 模式：数据契约优先 → Init 注册 → Task 调度。
 * app 把电机视为传递函数为 1 的设备，只给参考值，不碰闭环。
 * 基于 26_33 提炼，抄模式不抄参数。
 */

//app
#include "chassis.h"
#include "robot_def.h"
//module
#include "dji_motor.h"
#include "robot_bus.h"
//bsp
#include "bsp_dwt.h"

/* ============ 数据契约（定义在 robot_def.h，先定契约再写实现） ============ */
/*
typedef struct {
    float vx, vy, wz;              // 期望速度
    Chassis_Mode_e chassis_mode;   // ZERO_FORCE / FOLLOW_GIMBAL / ROTATE
    float offset_angle;
} Chassis_Ctrl_Cmd_s;

typedef struct {
    float real_vx, real_vy;
} Chassis_Upload_Data_s;
*/

/* ============ 实例与命令/反馈缓冲 ============ */
static DJIMotorInstance *motor_lf;              /* 其余三个电机同理 */
static Chassis_Ctrl_Cmd_s    chassis_cmd_recv;  /* 来自 cmd 的命令 */
static Chassis_Upload_Data_s chassis_feedback;  /* 回传给 cmd 的反馈 */

/* ============ Init：注册电机，app 不碰闭环 ============ */
void ChassisInit(void)
{
    Motor_Init_Config_s cfg = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x201,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 6, .Ki = 0, .Kd = 0,
                .MaxOut = 12000,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .outer_loop_type        = SPEED_LOOP,
            .close_loop_type        = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag     = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };

    motor_lf = DJIMotorRegister(&cfg);
    /* 其余三个：改 tx_id / motor_reverse_flag 后重复 DJIMotorRegister，同理 */
}

/* ============ Task：读命令 → 安全决策 → 解算 → 输出 → 回传反馈 ============ */
void ChassisTask(void)
{
    RobotBusReadChassisCmd(&chassis_cmd_recv);   /* cmd 写、子系统读 */

    /* 安全：ZERO_FORCE 停电机；其余模式才使能 */
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        DJIMotorStop(motor_lf);
        return;
    }
    DJIMotorEnable(motor_lf);

    /* 麦轮解算：vx/vy/wz → 轮速（四个轮各算各的） */
    float vt_lf = chassis_cmd_recv.vx - chassis_cmd_recv.vy - chassis_cmd_recv.wz;

    /* 输出：app 只给参考值，闭环在模块内部由 motor_task 驱动 */
    DJIMotorSetRef(motor_lf, vt_lf);

    /* 反馈回传：子系统写 Feed，cmd 读 */
    chassis_feedback.real_vx = vt_lf;
    RobotBusSendChassisFeed(&chassis_feedback);
}
