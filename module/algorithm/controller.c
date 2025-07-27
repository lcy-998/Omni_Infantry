#include "controller.h"
#include <math.h>
#include <memory.h>
#include <stdlib.h>

#define sign(x) (x > 0 ? 1 : (-1))

//电机堵转检测

PIDInstance *PIDRegister(PID_Init_Config_s *config)
{
    PIDInstance *instance = (PIDInstance *)malloc(sizeof(PIDInstance));
    memset(instance, 0, sizeof(PIDInstance));
    
    instance->Kp = config->Kp;
    instance->Ki = config->Ki;
    instance->Kd = config->Kd;

    instance->MaxOut = config->MaxOut;
    instance->DeadBand = config->DeadBand;

    instance->CoefA = config->CoefA;
    instance->CoefB = config->CoefB;
    instance->Improve = config->Improve;
    instance->IntegralLimit = config->IntegralLimit;
    instance->Output_LPF_RC = config->Output_LPF_RC;
    instance->Derivative_LPF_RC = config->Derivative_LPF_RC;

    instance->dt = 0.001;

    return instance;
}

float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    
    pid->Last_Measure = pid->Measure;
    pid->Last_Err = pid->Err;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;

    pid->Measure = measure;
    pid->ref = ref;
    pid->Err = pid->ref - pid->Measure;


    if(my_abs(pid->Err) > pid->DeadBand)
    {
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        if(pid->Improve & PID_Trapezoid_Integral)
        {
            pid->ITerm = pid->Ki * (pid->Err + pid->Last_Err) * pid->dt / 2;
        }
        else
        {
            pid->ITerm = pid->Ki * pid->Err * pid->dt;
        }

        if(pid->Improve & PID_ChangingIntegrationRate)
        {
            if(my_abs(pid->Err) > pid->CoefB && my_abs(pid->Err) < (pid->CoefA + pid->CoefB))
            {
                pid->ITerm *= (pid->CoefA - my_abs(pid->Err) + pid->CoefB) / pid->CoefA;
            }
            else if(my_abs(pid->Err) > (pid->CoefA + pid->CoefB))
            {
                pid->ITerm = 0;
            }
        }

        if(pid->Improve & PID_Derivative_On_Measurement)
        {
            pid->Pout = pid->Kp * (pid->Last_Measure - pid->Measure) / pid->dt;
        }
        else
        {
            pid->Pout = pid->Kp * pid->Err;
        }

        if(pid->Improve & PID_DerivativeFilter)
        {
            pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                        pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
        }

        pid->Iout += pid->ITerm;

        if(pid->Improve & PID_Integral_Limit)
        {
            if(my_abs(pid->Iout) > pid->IntegralLimit)
            {
                pid->Iout = pid->IntegralLimit * sign(pid->Iout);
            }
        }

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        if(pid->Improve & PID_OutputFilter)
        {
            pid->Output = pid->Output * pid->dt / (pid->dt + pid->Output_LPF_RC) +
                            pid->Last_Output * pid->Output_LPF_RC / (pid->dt + pid->Output_LPF_RC);
        }

        if(my_abs(pid->Output) > pid->MaxOut)
        {
            pid->Output = pid->MaxOut * sign(pid->Output);
        }
    }
    else
    {
        pid->Output = 0;
    }
    return pid->Output;
}
