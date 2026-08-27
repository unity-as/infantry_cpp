#ifndef PID_TEST_H
#define PID_TEST_H

#include <stdint.h>

typedef enum
{
    PID_MODE_POSITION,
    PID_MODE_DELTA
} PID_MODE;

typedef enum
{
    PID_FEATURE_NONE           = 0b00000000u,
    PID_FEATURE_INTEGRAL_LIMIT = 0b00000001u,
    PID_FEATURE_DERIVATIVE_LIMIT = 0b00000010u,
    PID_FEATURE_OUTPUT_LIMIT   = 0b00000100u,
    PID_FEATURE_DEAD_ZONE      = 0b00001000u,
    PID_FEATURE_FILTER         = 0b00010000u,
    PID_FEATURE_VARIABLE_GAIN  = 0b00100000u,
    PID_FEATURE_DERIVATIVE_ON_MEASUREMENT = 0b01000000u,
    PID_FEATURE_FEED_FORWARD     =0b10000000u,
} PID_FEATURE;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float setpoint;
    float integral;
    float last_feedback;
    float last_2_feedback;
    float last_error;
    float last_2_error;

    float output;

    float integral_limit;
    float derivative_limit;
    float output_min;
    float output_max;
    float dead_zone;

    float filter_alpha;

    float kp_extra;
    float kd_extra;

    PID_MODE mode;
    PID_FEATURE features;
    uint8_t period;
} PID_Instance;

typedef struct
{
    float kp;
    float ki;
    float kd;

    PID_MODE mode;
    PID_FEATURE features;

    float integral_limit;
    float derivative_limit;
    float output_min;
    float output_max;
    float dead_zone;

    float filter_alpha;

    float kp_extra;
    float kd_extra;

    uint8_t period;
} PID_Init_Config_s;

void PID_Set_Parameters(PID_Instance *instance, float kp, float ki, float kd);

#endif