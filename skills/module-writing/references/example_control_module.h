#ifndef DJI_MOTOR_EXAMPLE_H
#define DJI_MOTOR_EXAMPLE_H

#include <stdint.h>
#include "bsp_can.h"
#include "daemon.h"
#include "pid.h"

#define DJIM_MAX_INSTANCE 12

/* ===== Instance：状态归模块，无下划线 ===== */
typedef struct {
    CANInstance    *can;      /* 组合的子模块（bsp 实例在模块内部） */
    DaemonInstance *daemon;   /* 失联守护 */
    PIDInstance    *pid;      /* 控制环 */
    float measure;            /* 反馈值 */
    float target;             /* 目标值 */
    uint8_t id;
    uint8_t enabled;          /* 使能状态，注册≠启动 */
} DJIMotorInstance;

/* ===== Config：由 app 传入，板级细节集中于此 ===== */
typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint8_t id;
    PID_Init_Config_s pid_cfg;
} DJIMotor_Init_Config_s;

/* ===== API：Register 是唯一入口 ===== */
DJIMotorInstance *DJIMotorRegister(DJIMotor_Init_Config_s *cfg);
void DJIMotorSetRef(DJIMotorInstance *m, float ref);
void DJIMotorEnable(DJIMotorInstance *m);
void DJIMotorStop(DJIMotorInstance *m);
void DJIMotorControl(void);   /* 由 motor_task 1kHz 调用 */

#endif /* DJI_MOTOR_EXAMPLE_H */
