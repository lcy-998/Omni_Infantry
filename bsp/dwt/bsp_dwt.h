#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "main.h"

typedef struct 
{
    uint32_t s;
    uint32_t ms;
    uint32_t us;
}DWT_Time_s;


void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
void DWT_SysTimeUpdate(void);
float DWT_GerTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);



#endif
