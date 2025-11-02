#include "stdlib.h"
#include "stdbool.h"
#include "memory.h"
#include "buzzer.h"
#include "cmsis_os2.h"

extern osThreadId_t BuzzerHandle;
static BuzzerInstance *buzzer = NULL;
const uint16_t Note_Freq[] = {0,
                              16, 17, 18, 19, 21, 22, 23, 25, 26, 28, 29, 31,
                              33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 61,
                              65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123,
                              131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247,
                              262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
                              523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
                              1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
                              2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,
                              4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902};

void BuzzerRegister(void)
{
    BuzzerInstance *buzzer_ins = (BuzzerInstance *)malloc(sizeof(BuzzerInstance));
    memset(buzzer_ins, 0, sizeof(BuzzerInstance));
    PWM_Init_Config_s buzzer_pwm_config = {
        .handle = &htim4,
        .channel = TIM_CHANNEL_3,
        .dutyratio = 0,
        .period = 0.001f,
        .pwm_module_callback = NULL,
        .id = NULL,
    };
    buzzer_ins->buzzer_pwm = PWMRegister(&buzzer_pwm_config);
    buzzer_ins->sound = NULL;
    buzzer_ins->repeat = false;
    buzzer_ins->note_mode = NORMAL;
    buzzer_ins->octave = 4;
    buzzer_ins->tempo = 120;
    buzzer_ins->note_length = 4;
    buzzer_ins->next_tone = NULL;
    buzzer_ins->note = 0;
    buzzer_ins->busy = 0;
    buzzer = buzzer_ins;
}

void BuzzerPlay(char *sound)
{
    if(!buzzer)
    {
        BuzzerRegister();
    }
    if(buzzer->busy) return;
    buzzer->sound = sound;
    buzzer->next_tone = sound;
    buzzer->busy = 1;
    osThreadResume(BuzzerHandle);
}

void buzzer_silence(void)
{
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
}

void buzzer_one_note(uint16_t Note, float delay)
{
    if(!buzzer)
    {
        BuzzerRegister();
    }
    PWMSetPeriod(buzzer->buzzer_pwm, 1.0f / (float)Note);
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0.5f);
    float note_delay = 1000 * (4.0f / (float)buzzer->note_length) * 60 / (float)buzzer->tempo;
    note_delay += note_delay / 2.0f * (float)buzzer->dots;
    osDelay((uint32_t)note_delay);
    PWMSetDutyRatio(buzzer->buzzer_pwm, 0);
    osDelay(10);
}
