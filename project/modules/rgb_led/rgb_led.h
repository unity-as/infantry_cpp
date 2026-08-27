#ifndef RGB_LED_H
#define RGB_LED_H

#include "bsp_pwm.h"

typedef struct {
    PWM_Instance *r;
    PWM_Instance *g;
    PWM_Instance *b;
} RGB_Instance;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch_r;
    uint32_t ch_g;
    uint32_t ch_b;
} RGB_Init_Config_s;

RGB_Instance *RGB_Register(RGB_Init_Config_s *config);
RGB_Instance *RGB_Init(void);
void RGB_Set(RGB_Instance *rgb, uint16_t r, uint16_t g, uint16_t b);

#endif
