#include "rgb_led.h"

#include <stdlib.h>
#include <string.h>

RGB_Instance *RGB_Init(void)
{
    RGB_Init_Config_s config = {
        .htim = &htim5,
        .ch_r = TIM_CHANNEL_3,   // PH12
        .ch_g = TIM_CHANNEL_2,   // PH11
        .ch_b = TIM_CHANNEL_1,   // PH10
    };
    return RGB_Register(&config);
}

RGB_Instance *RGB_Register(RGB_Init_Config_s *config)
{
    RGB_Instance *rgb = (RGB_Instance *)malloc(sizeof(RGB_Instance));
    if (!rgb) return NULL;
    memset(rgb, 0, sizeof(RGB_Instance));

    PWM_Init_Config_s cfg_r = { .htim = config->htim, .channel = config->ch_r };
    PWM_Init_Config_s cfg_g = { .htim = config->htim, .channel = config->ch_g };
    PWM_Init_Config_s cfg_b = { .htim = config->htim, .channel = config->ch_b };

    rgb->r = PWM_Register(&cfg_r);
    rgb->g = PWM_Register(&cfg_g);
    rgb->b = PWM_Register(&cfg_b);

    if (!rgb->r || !rgb->g || !rgb->b) {
        free(rgb);
        return NULL;
    }
    return rgb;
}

void RGB_Set(RGB_Instance *rgb, uint16_t r, uint16_t g, uint16_t b)
{
    if (!rgb) return;
    PWM_Set_DutyRatio(rgb->r, (float)r / 999.0f);
    PWM_Set_DutyRatio(rgb->g, (float)g / 999.0f);
    PWM_Set_DutyRatio(rgb->b, (float)b / 999.0f);
}
