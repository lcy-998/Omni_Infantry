#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "stdint.h"
#include "ins_task.h"

#define HALF_TRACK_WIDTH 0.226   //半轮距 m
#define WHEEL_RADIUS 0.077       //轮半径 m

#define WHEEL_SPEED_MX (WHEEL_RADIUS * M3508_MX_APS / M3508_RATIO)  //轮组的最大速度
#define CHASSIS_APS_MX (WHEEL_SPEED_MX / HALF_TRACK_WIDTH)          //底盘最大角速度（角度制）

#define YAW_CHASSIS_ALIGN_ECD 4144
#define YAW_ECD_GREATER_THAN_4096 1
#define PITCH_HORIZON_ECD 0
#define PITCH_POS_UP_LIMIT_ECD 2048
#define PITCH_POS_DOWN_LIMIT_ECD 6154

#define BLOCK_CURRENT 9900
#define REVERSE_BLOCK_CURRENT (-2000)

/*-------------------BMI088在C板安装方向----------------*/

/*
 *橡胶板一侧为z轴负方向
 *PWM引脚一侧为x负正方向
 *CAN口一侧为y轴负方向
*/

/*-----------------------模式--------------------------*/

//底盘模式

typedef enum
{
    CHASSIS_ZERO_FORCE = 0,
    CHASSIS_ROTATE,
    CHASSIS_NO_FOLLOW,
    CHASSIS_FOLLOW
}Chassis_Mode_e;

//云台模式

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_FREE_MODE
}Gimbal_Mode_e;

//发射机构模式

typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
}Shoot_Mode_e;

typedef enum
{
    FRICTION_OFF = 0,
    FRICTION_ON,
}Friction_Mode_e;

typedef enum
{
    LOAD_STOP = 0,
    LOAD_REVERSE,
    LOAD_SINGLE_BULLET,
    LOAD_BURSTFIRE,
}Load_Mode_e;

/*-----------------------发送--------------------------*/

//底盘控制

typedef struct 
{
    float vx;
    float vy;
    float wz;
    float offset_angle;
    Chassis_Mode_e chassis_mode;
}Chassis_Ctrl_Cmd_s;

typedef struct
{
    float yaw;
    float pitch;
    float chassis_rotate_wz;
    Gimbal_Mode_e gimbal_mode;
}Gimbal_Ctrl_Cmd_s;

typedef struct 
{
    Shoot_Mode_e shoot_mode;
    Load_Mode_e load_mode;
    Friction_Mode_e firction_mode;
    uint8_t rest_heat;
    uint16_t shooter_heat_cooling_rate;
    uint16_t shoter_referee_heat;
    uint16_t shoter_cooling_limit;
    float shoot_rate;
    float bullet_speed;
}Shoot_Ctrl_Cmd_s;


/*-----------------------反馈--------------------------*/

/*底盘反馈*/

typedef struct 
{
    float vx;
    float vy;
    float wz;

}Chassis_Upload_Data_s;


/*云台反馈*/

typedef struct 
{
    float yaw;
    float yaw_total_angle;
    float yaw_motor_single_round_angle;
    
}Gimbal_Upload_Data_s;

typedef struct 
{
    int shoot_heat_control;
    float shooter_local_heat;
    
}Shoot_Upload_Data_s;



#endif
