#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include "tim.h"
#include "stm32f407xx.h"

#define PWM_MX_REGISTER_CNT 16

typedef struct _PWMInstance
{
    TIM_HandleTypeDef *handle;
    uint32_t channel;
    uint32_t tclk;
    float period;
    float dutyratio;
    void (*pwm_module_callback)(struct _PWMInstance*);
    void *id;
}PWMInstance;

typedef struct
{
    TIM_HandleTypeDef *handle;
    uint32_t channel;
    float period;//此处是真正的周期，不是ARR的值（ARR的值为period * (tclk / (psc + 1))）
    float dutyratio;
    void (*pwm_module_callback)(PWMInstance *);
    void *id;
}PWM_Init_Config_s;

void PWMSetPeriod(PWMInstance *instance, float period);
void PWMSetDutyRatio(PWMInstance *instance, float dutyratio);
PWMInstance *PWMRegister(PWM_Init_Config_s *config);
void PWMStart(PWMInstance *instance);
void PWMStop(PWMInstance *instance);
void PWMStartDMA(PWMInstance *instance, uint32_t *pData, uint32_t size);

#endif
