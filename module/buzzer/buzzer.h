#ifndef BUZZER_H
#define BUZZER_H

#include "bsp_pwm.h"

#define Do_freq Note_Freq[49]
#define Re_freq Note_Freq[51]
#define Mi_freq Note_Freq[53]
#define Fa_freq Note_Freq[54]
#define So_freq Note_Freq[56]
#define La_freq Note_Freq[58]
#define Si_freq Note_Freq[60]

typedef enum
{
    NORMAL,
    LEGATO,
    STACCATO
}NoteMode_e;

typedef struct 
{
    PWMInstance *buzzer_pwm;
    char *sound;
    char *next_tone;
    NoteMode_e note_mode;
    unsigned note_length;
    unsigned dots;
    unsigned octave;
    unsigned tempo;
    uint8_t repeat;
    unsigned note;
    uint8_t busy;
}BuzzerInstance;




#endif
