#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "controller.h"
#include "bsp_can.h"

#define M3508_RATIO 19.203208f
#define M3508_MX_APS 54000.0f

typedef enum 
{
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP = 0b0010,
    ANGLE_LOOP = 0b0100,

    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THREE_LOOP = 0b0111

}Closeloop_Type_e;

typedef enum
{
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01,
    SPEED_FEEDFORWARD = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD
}Feedforward_Type_e;

typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED
}Feedback_Sourse_e;

typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1
}Motor_Reverse_Flag_e;

typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
}Feedback_Reverse_Flag_e;

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENABLE = 1
}Motor_Working_Type_e;

typedef struct 
{
    Closeloop_Type_e close_loop;

    Motor_Reverse_Flag_e motor_reverse_flag;
    Feedback_Reverse_Flag_e feedback_reverse_flag;

    Feedback_Sourse_e angle_feedback_source;
    Feedback_Sourse_e speed_feedback_source;

    Feedforward_Type_e feedforward_flag;
}Motor_Setting_s;

typedef struct 
{
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    PIDInstance *current_pid;
    PIDInstance *speed_pid;
    PIDInstance *angle_pid;

    float pid_ref;
}Motor_Controller_s;

typedef struct
{
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    PID_Init_Config_s current_pid_init;
    PID_Init_Config_s speed_pid_init;
    PID_Init_Config_s angle_pid_init;

}Motor_Controller_Init_s;


typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04
}Motor_Type_e;

typedef struct 
{
    Motor_Controller_Init_s motor_controller_init;
    Motor_Setting_s motor_setting;
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;
    
}Motor_Init_Config_s;


#endif
