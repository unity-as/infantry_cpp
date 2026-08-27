#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include "adc.h" // CubeMX 生成头,提供 ADC_HandleTypeDef

#define ADC_MX_DEVICE_NUM 8 // 最大支持的ADC实例数量

/* ADC实例 */
typedef struct
{
    ADC_HandleTypeDef *adc_handle; // adc句柄
    float vref;                    // 参考电压 V
    uint16_t full_scale;           // 满量程原始值,由 Init.Resolution 自动推导
} ADC_Instance;

/* ADC初始化模板 */
typedef struct
{
    ADC_HandleTypeDef *adc_handle; // adc句柄
    float vref;                    // 参考电压 V
} ADC_Init_Config_s;

ADC_Instance *ADCRegister(ADC_Init_Config_s *config);
uint16_t ADCGetRaw(ADC_Instance *inst);
float ADCGetVoltage(ADC_Instance *inst);

#endif
