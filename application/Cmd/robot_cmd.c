#include "robot_def.h"
#include "robot_cmd.h"

#include "message_center.h"
#include "remote_control.h"
#include "motor_def.h"
#include "djimotor.h"
#include "slope.h"

#include "bsp_usart.h"

#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)

static Publisher_s *chassis_cmd_pub;
static Subscriber_s *chassis_feedback_sub;

static Chassis_Ctrl_Cmd_s chassis_cmd_send;
static Chassis_Upload_Data_s chassis_fetch_data;

static RC_Ctrl_t *rc_data;

static Publisher_s *gimbal_cmd_pub;
static Subscriber_s *gimbal_feedback_sub;

static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
static Gimbal_Upload_Data_s gimbal_fetch_data;

static Publisher_s *shoot_cmd_pub;
static Subscriber_s *shoot_feedback_sub;

static Shoot_Ctrl_Cmd_s shoot_cmd_send;
static Shoot_Upload_Data_s shoot_fetch_data;

static Slope_t vx_slope, vy_slope, wz_slope;


void RobotCmdInit(void)
{
    rc_data = RemoteControlInit(&huart3);

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feedback_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feedback_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));    

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feedback_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

    SlopeInit(&vx_slope, 1, 1);
    SlopeInit(&vy_slope, 1, 1);
    SlopeInit(&wz_slope, 2, 2);
}

static void CalcOffsetAngle(void)
{
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle;

#if YAW_ECD_GREATER_THAN_4096
    if(angle >= YAW_ALIGN_ANGLE - 180.0 && angle <= YAW_ALIGN_ANGLE + 180.0)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0;
#else
    if(angle >= YAW_ALIGN_ANGLE - 180.0 && angle <= YAW_ALIGN_ANGLE + 180.0)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0;
#endif
}

static void RemoteControlSet(void)
{
    switch (rc_data[TEMP].rc.sl)
    {
    case RC_SW_UP:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        break;
    case RC_SW_MID:
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        break;
    case RC_SW_DOWN:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        break;
    default:
        break;
    }
    gimbal_cmd_send.yaw += 0.001 * 50.0 * (float)rc_data[TEMP].rc.dial / 660.0;

    chassis_cmd_send.vx = SlopeUpdate(&vx_slope, WHEEL_SPEED_MX * (float)rc_data[TEMP].rc.ch1 / 660.0);
    chassis_cmd_send.vy = SlopeUpdate(&vy_slope, - WHEEL_SPEED_MX * (float)rc_data[TEMP].rc.ch0 / 660.0);
    chassis_cmd_send.wz = SlopeUpdate(&wz_slope, - CHASSIS_APS_MX * (float)rc_data[TEMP].rc.ch2 / 660.0);
    
}

void RobotCmdTask(void)
{
    SubGetMessage(chassis_feedback_sub, (void *)&chassis_fetch_data);
    SubGetMessage(gimbal_feedback_sub, (void *)&gimbal_fetch_data);

    CalcOffsetAngle();

    RemoteControlSet();

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}