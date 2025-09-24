#ifndef __BSP_INIT_H__
#define __BSP_INIT_H__

#include "bsp_dwt.h"

void BSPInit()
{
    DWT_Init(168);
}

#endif
