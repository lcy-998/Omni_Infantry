#include "bsp_pwm.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t idx;
static PWMInstance *pwm_instance_list[PWM_MX_REGISTER_CNT] = {NULL};

static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim)
{
    uintptr_t tclk_temp = ((uintptr_t)(htim->Instance));
    if(
            (tclk_temp <= (APB1PERIPH_BASE + 0X2000UL)) &&
            (tclk_temp >= (APB1PERIPH_BASE + 0X0000UL)))
        {
            return (HAL_RCC_GetPCLK1Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
        }
    else if (
        ((tclk_temp <= (APB2PERIPH_BASE + 0X0400UL)) &&
         (tclk_temp >= (APB2PERIPH_BASE + 0X0000UL))) ||
        ((tclk_temp <= (APB2PERIPH_BASE + 0X4800UL)) &&
         (tclk_temp >= (APB2PERIPH_BASE + 0X4000UL))))
         {
            return (HAL_RCC_GetPCLK2Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >>RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
         }
    return 0;
}

//ARR
void PWMSetPeriod(PWMInstance *instance, float period)
{
    instance->period = period;
    __HAL_TIM_SetAutoreload(instance->handle, period * ((instance->tclk)/(instance->handle->Init.Prescaler + 1)));
}

//CCR
void PWMSetDutyRatio(PWMInstance *instance, float dutyratio)
{
    instance->dutyratio = dutyratio;
    __HAL_TIM_SetCompare(instance->handle, instance->channel, dutyratio * (instance->handle->Instance->ARR));
}

PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    PWMInstance *instance = (PWMInstance *)malloc(sizeof(PWMInstance));
    memset(instance, 0, sizeof(PWMInstance));

    instance->handle = config->handle;
    instance->channel = config->channel;
    instance->period = config->period;
    instance->dutyratio = config->dutyratio;
    instance->pwm_module_callback = config->pwm_module_callback;
    instance->id = config->id;
    instance->tclk = PWMSelectTclk(instance->handle);

    HAL_TIM_PWM_Start(instance->handle, instance->channel);
    PWMSetPeriod(instance, instance->period);
    PWMSetDutyRatio(instance, instance->dutyratio);
    pwm_instance_list[idx++] = instance;
    return instance;
}

void PWMStart(PWMInstance *instance)
{
    HAL_TIM_PWM_Start(instance->handle, instance->channel);
}

void PWMStop(PWMInstance *instance)
{
    HAL_TIM_PWM_Stop(instance->handle, instance->channel);
}

void PWMStartDMA(PWMInstance *instance, uint32_t *pData, uint32_t size)
{
    HAL_TIM_PWM_Start_DMA(instance->handle, instance->channel, pData, size);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for(uint8_t i = 0; i < idx; i++)
    {
        if(pwm_instance_list[i]->handle == htim && htim->Channel == (1 << (pwm_instance_list[i]->channel / 4)))
        {
            if(pwm_instance_list[i]->pwm_module_callback)
            {
                pwm_instance_list[i]->pwm_module_callback(pwm_instance_list[i]);
            }
            return;
        }
    }
}
