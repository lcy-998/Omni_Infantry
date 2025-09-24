#include "bsp_dwt.h"
#include "cmsis_os.h"

static DWT_Time_s SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RoundCount;
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;

static uint8_t DWT_CNT_Updata(void)
{
    static volatile uint8_t bit_locker = 0;
    if(!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if(cnt_now < CYCCNT_LAST)
        {
            CYCCNT_RoundCount++;
            CYCCNT_LAST = DWT->CYCCNT;
            return 1;
        }
        CYCCNT_LAST = DWT->CYCCNT;
        bit_locker = 0;
    }
    return 0;
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = (uint32_t)0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;

    CYCCNT_RoundCount = 0;

    DWT_CNT_Updata();
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt;
    dt = ((float)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz)); 
    *cnt_last = cnt_now;

    DWT_CNT_Updata();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Updata();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_Temp1, CNT_Temp2, CNT_Temp3;

    DWT_CNT_Updata();

    CYCCNT64 = (uint64_t)CYCCNT_RoundCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_Temp1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_Temp2 = CYCCNT64 - CNT_Temp1 * CPU_FREQ_Hz;
    SysTime.s = CNT_Temp1;
    SysTime.ms = CNT_Temp2 / CPU_FREQ_Hz_ms;
    CNT_Temp3 = CNT_Temp2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_Temp3 / CPU_FREQ_Hz_us;
}

float DWT_GerTimeline_s(void)
{
    DWT_SysTimeUpdate();
    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;
    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();
    float DWT_Timelinef32 = 1000.0f * SysTime.s + SysTime.ms + SysTime.us * 0.001f;
    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    uint64_t DWT_Timeline = 1000000 * SysTime.s + 1000 * SysTime.ms + SysTime.us;
    return DWT_Timeline;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while((float)(DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz);
}
