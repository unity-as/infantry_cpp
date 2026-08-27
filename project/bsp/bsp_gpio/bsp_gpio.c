#include "bsp_gpio.h"
#include <stdlib.h>
#include <string.h>

static uint8_t idx=0;
static GPIO_Instance *gpio_instance[GPIO_MX_DEVICE_NUM];

GPIO_Instance *GPIO_Register(GPIO_Init_Config_s *GPIO_config)
{
    if(idx >= GPIO_MX_DEVICE_NUM)
        return NULL;

    GPIO_Instance *instance=(GPIO_Instance*)malloc(sizeof(GPIO_Instance));
    memset(instance,0,sizeof(GPIO_Instance));
    instance->GPIOx=GPIO_config->GPIOx;
    instance->GPIO_Pin=GPIO_config->GPIO_Pin;
    instance->PinState=GPIO_config->PinState;
    instance->exti_mode=GPIO_config->exti_mode;
    gpio_instance[idx++]=instance;
    return instance;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIO_Instance *gpio;
    for(size_t i=0;i<idx;i++)
    {
        gpio=gpio_instance[i];
        if(gpio->GPIO_Pin==GPIO_Pin&&gpio->gpio_callback!=NULL)
        {
            gpio->gpio_callback(gpio);
            return;
        }
    }
}

GPIO_PinState GPIO_ReadPin(GPIO_Instance *gpio)
{
    return HAL_GPIO_ReadPin(gpio->GPIOx,gpio->GPIO_Pin);
}

void GPIO_WritePin(GPIO_Instance *gpio,GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(gpio->GPIOx,gpio->GPIO_Pin , PinState);
}

void GPIO_TogglePin(GPIO_Instance *gpio)
{
    HAL_GPIO_TogglePin(gpio->GPIOx,gpio->GPIO_Pin);
}