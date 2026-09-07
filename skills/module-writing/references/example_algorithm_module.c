#include "pid.h"

#include <stdlib.h>
#include <string.h>

PIDInstance *PIDRegister(PID_Init_Config_s *cfg)
{
    PIDInstance *p = malloc(sizeof(*p));
    if (!p) return NULL;
    memset(p, 0, sizeof(*p));
    p->kp = cfg->kp;
    p->ki = cfg->ki;
    p->kd = cfg->kd;
    p->integral_limit = cfg->integral_limit;
    p->output_max = cfg->output_max;
    p->output_min = cfg->output_min;
    return p;
}

void PID_Set_Setpoint(PIDInstance *p, float setpoint)
{
    if (p) p->setpoint = setpoint;
}

float PID_Update(PIDInstance *p, float feedback)
{
    float err = p->setpoint - feedback;
    p->integral += err;
    if (p->integral > p->integral_limit) p->integral = p->integral_limit;
    if (p->integral < -p->integral_limit) p->integral = -p->integral_limit;

    p->out = p->kp * err + p->ki * p->integral + p->kd * (err - p->last_error);
    p->last_error = err;

    if (p->out > p->output_max) p->out = p->output_max;
    if (p->out < p->output_min) p->out = p->output_min;
    return p->out;
}

void PID_Reset(PIDInstance *p)
{
    if (!p) return;
    p->integral = 0;
    p->last_error = 0;
    p->out = 0;
}
