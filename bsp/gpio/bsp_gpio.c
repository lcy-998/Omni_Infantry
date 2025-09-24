#include "bsp_gpio.h"
#include "stdlib.h"
#include "memory.h"

static GPIOInstance *gpio_instance_list[GPIO_MX_REGISTER_CNT] = {0};
static uint8_t idx = 0;

GPIOInstance *GPIORegister(GPIO_Init_Config_s *config)
{
    GPIOInstance *instance = (GPIOInstance *)malloc(sizeof(GPIOInstance));
    memset(instance, 0, sizeof(GPIOInstance));

    instance->gpio = config->gpio;
    instance->gpio_exti_mode = config->gpio_exti_mode;
    instance->gpio_module_callback = config->gpio_module_callback;
    instance->gpio_pin = config->gpio_pin;
    instance->id = config->id;

    return instance;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for(size_t i = 0; i < idx; i++)
    {
        if(gpio_instance_list[i]->gpio_pin = GPIO_Pin)
        {
            if(gpio_instance_list[i]->gpio_module_callback != NULL)
            {
                gpio_instance_list[i]->gpio_module_callback(gpio_instance_list[i]);
            }
            return;
        }
    }
}
