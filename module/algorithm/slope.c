#include "slope.h"

#define my_abs(x) ((x > 0) ? (x) : -(x))

void SlopeInit(Slope_t *slope, float _increase_value, float _decrease_value)
{
    slope->now = 0;
    slope->out = 0;
    slope->target = 0;
    slope->increase_value = _increase_value;
    slope->decrease_value = _decrease_value;
}

float SlopeUpdate(Slope_t *slope, float _target)
{
    slope->target = _target;
    if(slope->now > 0)
    {
        if(slope->target > slope->now)
        {
            if(my_abs(slope->now > slope->target) > slope->increase_value)
            {
                slope->out += slope->increase_value;
            }
            else
            {
                slope->out = slope->target;
            }
        }
        else if(slope->target < slope->now)
        {
            if (my_abs(slope->now - slope->target) > slope->decrease_value)
            {
                slope->out -= slope->decrease_value;
            }
            else
            {
                slope->out = slope->target;
            }
        }
    }
    else if(slope->now < 0)
    {
        if(slope->target < slope->now)
        {
            if(my_abs(slope->target - slope->now) > slope->increase_value)
            {
                slope->out -= slope->increase_value;
            }
            else
            {
                slope->out = slope->target;
            }
        }
        else if(slope->target > slope->now)
        {
            if(my_abs(slope->target - slope->now) > slope->decrease_value)
            {
                slope->out += slope->decrease_value;
            }
            else
            {
                slope->out = slope->target;
            }
        }
    }
    else
    {
        if(slope->target > slope->now)
        {
            if(my_abs(slope->now - slope->target) > slope->increase_value)
            {
                slope->out += slope->increase_value;
            }
            else
            {
                slope->out = slope->target;
            }
        }
        else if (slope->target < slope->now)
        {
            if(my_abs(slope->now - slope->target) > slope->decrease_value)
            {
                slope->out -= slope->increase_value;
            }
            else
            {
                slope->out = slope->target;
            }
        }
    }

    slope->now = slope->out;
    return slope->out;
}
