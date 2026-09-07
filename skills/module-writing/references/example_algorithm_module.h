#ifndef PID_EXAMPLE_H
#define PID_EXAMPLE_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integral;
    float last_error;
    float out;
} PIDInstance;

typedef struct {
    float kp, ki, kd;
    float integral_limit;
    float output_max, output_min;
} PID_Init_Config_s;

PIDInstance *PIDRegister(PID_Init_Config_s *cfg);
void PID_Set_Setpoint(PIDInstance *p, float setpoint);
float PID_Update(PIDInstance *p, float feedback);
void PID_Reset(PIDInstance *p);

#endif /* PID_EXAMPLE_H */
