#include "bsp_adc.h"

#include <stdlib.h>
#include <string.h>

#define ADC_TIMEOUT_MS 10 // 手动读取模式下的转换超时,单位 ms

static ADC_Instance *adc_instance[ADC_MX_DEVICE_NUM] = {0}; // 所有的 adc instance 保存于此
static uint8_t idx = 0;

// 根据 Init.Resolution 的位宽编码推导满量程原始值
// (ADC_RESOLUTION_12B=0, 10B/8B/6B 是 CR1 的位编码而非位数,故用 switch 而非算术)
static uint16_t ADC_FullScale(uint32_t resolution)
{
    switch (resolution)
    {
        case ADC_RESOLUTION_12B: return 4095;
        case ADC_RESOLUTION_10B: return 1023;
        case ADC_RESOLUTION_8B:  return 255;
        case ADC_RESOLUTION_6B:  return 63;
        default:                 return 4095;
    }
}

ADC_Instance *ADCRegister(ADC_Init_Config_s *config)
{
    if (config == NULL || config->adc_handle == NULL)
        return NULL;
    if (idx >= ADC_MX_DEVICE_NUM)
        return NULL; // 超过最大实例数

    ADC_Instance *instance = (ADC_Instance *)malloc(sizeof(ADC_Instance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(ADC_Instance));

    instance->adc_handle = config->adc_handle;
    instance->vref = config->vref;
    instance->full_scale = ADC_FullScale(config->adc_handle->Init.Resolution);

    adc_instance[idx++] = instance; // 将实例保存到列表中,便于调试直接检查
    return instance;
}

uint16_t ADCGetRaw(ADC_Instance *inst)
{
    if (inst == NULL || inst->adc_handle == NULL)
        return 0;

    HAL_ADC_Start(inst->adc_handle);
    HAL_ADC_PollForConversion(inst->adc_handle, ADC_TIMEOUT_MS);
    uint16_t raw = HAL_ADC_GetValue(inst->adc_handle);
    HAL_ADC_Stop(inst->adc_handle);

    return raw;
}

float ADCGetVoltage(ADC_Instance *inst)
{
    if (inst == NULL)
        return 0.0f;

    return (float)ADCGetRaw(inst) * inst->vref / (float)inst->full_scale;
}
