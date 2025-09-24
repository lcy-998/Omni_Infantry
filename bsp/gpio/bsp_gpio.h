#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "gpio.h"

#define GPIO_MX_REGISTER_CNT 10

typedef enum
{
    GPIO_EXTI_MODE_NONE = 0,
    GPIO_EXTI_MODE_RISING ,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING
}GPIO_EXTI_Mode_e;

typedef struct _GPIOInstance
{
    GPIO_TypeDef *gpio;
    GPIO_EXTI_Mode_e gpio_exti_mode;
    uint16_t gpio_pin;
    void (*gpio_module_callback)(struct _GPIOInstance *);
    void *id;
}GPIOInstance;

typedef struct
{
    GPIO_TypeDef *gpio;
    GPIO_EXTI_Mode_e gpio_exti_mode;
    uint16_t gpio_pin;
    void (*gpio_module_callback)(GPIOInstance *);
    void *id;
}GPIO_Init_Config_s;

GPIOInstance *GPIORegister(GPIO_Init_Config_s *config);


#endif
