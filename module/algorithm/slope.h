#ifndef __SLOPE_H
#define __SLOPE_H

typedef struct slope_t
{
    float now;
    float target;
    float out;
    float increase_value;
    float decrease_value;
}Slope_t;

void SlopeInit(Slope_t *slope, float _increase_value, float _decrease_value);
float SlopeUpdate(Slope_t *slope, float _target);

#endif
