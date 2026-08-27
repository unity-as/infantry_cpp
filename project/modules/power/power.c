#include "power.h"
#include "bsp_adc.h"

static ADC_Instance *power_adc = NULL;

void Power_Init(ADC_HandleTypeDef *hadc)
{
    ADC_Init_Config_s cfg = {
        .adc_handle = hadc,
        .vref = 3.3f, // ADC 参考电压 VDDA
    };
    power_adc = ADCRegister(&cfg);
}

float Power_GetBusVoltage(void)
{
    if (power_adc == NULL)
        return 0.0f;

    return ADCGetVoltage(power_adc) * POWER_VBAT_DIVIDER;
}
