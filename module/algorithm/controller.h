#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "main.h"
#include <math.h>

#define my_abs(x) ((x > 0) ? (x) : -(x))


typedef enum
{
    PID_IMPROVEMENT_NONE = 0b00000000,
    PID_Integral_Limit = 0b00000001,
    PID_Derivative_On_Measurement = 0b00000010,
    PID_Trapezoid_Integral = 0b00000100,
    PID_Proportional_On_Measure = 0b00001000,
    PID_OutputFilter = 0b00010000,
    PID_ChangingIntegrationRate = 0b00100000,
    PID_DerivativeFilter = 0b01000000,
    
    //PID错误信号
}PID_Improvement_e;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA;
    float CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float ref;

    float dt;

}PIDInstance;

typedef struct 
{
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA;
    float CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;
}PID_Init_Config_s;

PIDInstance *PIDRegister(PID_Init_Config_s *config);
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif
