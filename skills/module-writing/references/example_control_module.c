#include "dji_motor.h"

#include <stdlib.h>
#include <string.h>

static DJIMotorInstance *instances[DJIM_MAX_INSTANCE];
static uint8_t idx;

/* ===== 解码回调：bsp 还回 owner → 解析 → 喂狗 ===== */
static void Decode(CANInstance *can)
{
    DJIMotorInstance *m = (DJIMotorInstance *)can->device;
    m->measure = (float)((can->rx_buff[0] << 8) | can->rx_buff[1]);
    DaemonReload(m->daemon);
}

/* ===== 失联回调：超时后安全处理 ===== */
static void Lose(void *device)
{
    DJIMotorInstance *m = (DJIMotorInstance *)device;
    m->enabled = 0;   /* 失联即停，输出零 */
}

/* ===== Register 六步曲 ===== */
DJIMotorInstance *DJIMotorRegister(DJIMotor_Init_Config_s *cfg)
{
    if (cfg->id == 0 || idx >= DJIM_MAX_INSTANCE) return NULL;   /* 1 校验 */
    DJIMotorInstance *m = malloc(sizeof(*m));
    if (!m) return NULL;
    memset(m, 0, sizeof(*m));                                    /* 2 分配清零 */

    CAN_Init_Config_s can_cfg = {
        .can_handle          = cfg->can_handle,
        .rx_id               = 0x200 + cfg->id,
        .can_module_callback = Decode,
        .device              = m,                                /* owner 还原 */
    };
    Daemon_Init_Config_s daemon_cfg = {
        .owner_id     = m,
        .reload_count = 100,
        .callback     = Lose,
    };
    m->can    = CANRegister(&can_cfg);                           /* 3 组合子模块 */
    m->daemon = DaemonRegister(&daemon_cfg);
    m->pid    = PIDRegister(&cfg->pid_cfg);

    instances[idx++] = m;                                        /* 4 挂静态表 */
    m->enabled = 0;                                              /* 5 注册≠启动 */
    return m;                                                    /* 6 返回 */
}

/* ===== 控制任务：由 motor_task 1kHz 统一驱动 ===== */
void DJIMotorControl(void)
{
    for (uint8_t i = 0; i < idx; ++i) {
        DJIMotorInstance *m = instances[i];
        if (!m->enabled) { m->target = 0; continue; }            /* 未使能输出零 */
        PID_Set_Setpoint(m->pid, m->target);
        PID_Update(m->pid, m->measure);
        m->can->tx_buff[0] = (uint8_t)((int16_t)m->pid->out >> 8);
        m->can->tx_buff[1] = (uint8_t)((int16_t)m->pid->out & 0xFF);
        CANTransmit(m->can, 1);
    }
}

/* ===== API ===== */
void DJIMotorSetRef(DJIMotorInstance *m, float ref) { if (m) m->target = ref; }
void DJIMotorEnable(DJIMotorInstance *m)           { if (m) m->enabled = 1; }
void DJIMotorStop(DJIMotorInstance *m)             { if (m) m->enabled = 0; }
