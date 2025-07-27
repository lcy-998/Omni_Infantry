#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__I

#include "main.h"

#define LAST 1
#define TEMP 0

#define RC_CH_VALUE_MIN     ((uint16_t)364)
#define RC_CH_VALUE_OFFSET  ((uint16_t)1024)
#define RC_CH_VALUE_MAX     ((uint16_t)1684)

#define RC_SW_UP            ((uint16_t)1)
#define RC_SW_MID           ((uint16_t)3)
#define RC_SW_DOWN          ((uint16_t)2)

#define REMOTE_CONTROL_FRAME_SIZE 18u

typedef union 
{
    struct 
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;

        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys;
}Key_t;




typedef struct 
{
    struct
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t dial;

        uint8_t s1;
        uint8_t s2;
    }rc;

    struct 
    {
        uint16_t x;
        uint16_t y;
        uint16_t z;
        uint8_t press_l;
        uint8_t press_r;
    }mouse;
    
    Key_t key;
    
}RC_Ctrl_t;

RC_Ctrl_t *RemoteControlInit(UART_HandleTypeDef *handle);
uint8_t RemoteControlIsOnline(void);

#endif
