#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "chassis_motion.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum {
    CHASSIS_MODE_NO_ROTATION,
    CHASSIS_MODE_FOLLOW,
    CHASSIS_MODE_LITTLE_TOP,
} Chassis_Mode;

typedef struct {
    float v;
    float w_rot;
    float yaw_motor_angle;
    Chassis_Mode mode;
    uint8_t enable;
} Chassis_Cmd;

extern Chassis_Cmd chassis_cmd;
extern ChassisMotion_Instance *chassis_inst;

void Chassis_Init(void);
void Chassis_SetMode(Chassis_Mode mode);
void Chassis_SetPowerLimit(ChassisMotion_Instance *chassis, float limit);
float Chassis_GetPower(ChassisMotion_Instance *chassis);

#ifdef __cplusplus
}
#endif

#endif
