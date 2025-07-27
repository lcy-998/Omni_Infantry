#ifndef __DJIMOTOR_H__
#define __DJIMOTOR_H__

#include "motor_def.h"
#include <stdint.h>

#define DJI_MOTOR_CNT 12

#define ECD_ANGLE_COEF_DJI 0.43945f
#define SPEED_SMOOTH_COEF 1.0f
#define CURRENT_SMOOTH_COEF 0.90f

typedef struct
{
    uint16_t last_ecd;
    uint16_t ecd;
    float angle_single_round;
    float speed_aps;
    int16_t real_current;
    uint8_t temperature;

    float total_angle;
    int8_t total_round;
}DJI_Motor_Measures_s;

typedef struct 
{
    DJI_Motor_Measures_s measure;
    Motor_Setting_s motor_setting;
    Motor_Controller_s motor_controller;
    CANInstance *motor_can_instance;
    Motor_Type_e motor_type;
    Motor_Working_Type_e stop_flag;

    uint8_t sender_group;
    uint8_t message_num;

}DJIMotorInstance;

DJIMotorInstance *DJIMotorRegister(Motor_Init_Config_s *config);
void DJIMotorChangeFeed(DJIMotorInstance *instance, Closeloop_Type_e loop,Feedback_Sourse_e feedback_source);
void DJIMotorEnable(DJIMotorInstance *instance);
void DJIMotorStop(DJIMotorInstance *instance);
void DJIMotorSetLoop(DJIMotorInstance *instance, Closeloop_Type_e loop);
void DJIMotorSetRef(DJIMotorInstance *instance, float ref);
void DJIMotorControl(void);

#endif
