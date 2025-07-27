#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#define HALF_TRACK_WIDTH 0.30
#define WHEEL_RADIUS 0.05

#define WHEEL_SPEED_MX (WHEEL_RADIUS * M3508_MX_APS / M3508_RATIO) //轮组的最大速度
#define CHASSIS_APS_MX (WHEEL_SPEED_MX / HALF_TRACK_WIDTH)


typedef struct 
{
    float vx;
    float vy;
    float wz;

}Chassis_Ctrl_Cmd_s;

typedef struct 
{
    float vx;
    float vy;
    float wz;

}Chassis_Upload_Data_s;





#endif
