#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "gpio.h"
__GPIO_H__

#define GPIO_MX_DEVICE_NUM 20       

typedef enum
{
    GPIO_EXTI_MODE_NONE=0u,
    GPIO_EXTI_MODE_RISING,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
} GPIO_EXTI_MODE_e;

typedef struct gpio
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    GPIO_PinState PinState;
    GPIO_EXTI_MODE_e exti_mode;
    void (*gpio_callback)(struct gpio*)
} GPIO_Instance;

typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    GPIO_PinState PinState;
    GPIO_EXTI_MODE_e exti_mode
} GPIO_Init_Config_s;

GPIO_Instance *GPIO_Register(GPIO_Init_Config_s *GPIO_config);
GPIO_PinState GPIO_ReadPin(GPIO_Instance *gpio);
void GPIO_WritePin(GPIO_Instance *gpio, GPIO_PinState state);
void GPIO_TogglePin(GPIO_Instance *gpio);

#endif