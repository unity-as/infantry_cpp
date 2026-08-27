#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// 软件限位 (deg)
#define GIMBAL_PITCH_MIN  -30.7f
#define GIMBAL_PITCH_MAX   10.5f

typedef struct {
    float yaw;
    float pitch;
    uint8_t enable;
} Gimbal_Cmd;

extern Gimbal_Cmd gimbal_cmd;

void  Gimbal_Init(void);
void  Gimbal_Set_Increment(float yaw_delta, float pitch_delta);
void  Gimbal_Set_Increment_Setpoint(float yaw, float pitch);
void  Gimbal_Set_Mode(uint8_t yaw_mode, uint8_t pitch_mode);
void  Gimbal_Set_Velocity(float yaw_vel, float pitch_vel);
void  Gimbal_AutoAim(float yaw_err, float pitch_err);
void  Gimbal_AutoAim_Reset(void);
void  Gimbal_SyncTarget(void);
float Gimbal_GetYaw(void);

#ifdef __cplusplus
}
#endif

#endif
