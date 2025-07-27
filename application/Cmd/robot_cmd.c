#include "robot_def.h"
#include "robot_cmd.h"

#include "message_center.h"
#include "remote_control.h"
#include "motor_def.h"

#include "bsp_usart.h"

static Publisher_s *chassis_cmd_pub;
static Subscriber_s *chassis_feedback_sub;

static Chassis_Ctrl_Cmd_s chassis_cmd_send;
static Chassis_Upload_Data_s chassis_fetch_data;

static RC_Ctrl_t *rc_data;


void RobotCmdInit(void)
{
    rc_data = RemoteControlInit(&huart3);

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feedback_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));


}

static void RemoteControlSet(void)
{
    chassis_cmd_send.vx = WHEEL_SPEED_MX * (float)rc_data[TEMP].rc.ch0 / 660.0;
    chassis_cmd_send.vy = WHEEL_SPEED_MX * (float)rc_data[TEMP].rc.ch1 / 660.0;
    chassis_cmd_send.wz = CHASSIS_APS_MX * (float)rc_data[TEMP].rc.ch2 / 660.0;
}

void RobotCmdTask(void)
{
    SubGetMessage(chassis_feedback_sub, (void *)&chassis_fetch_data);


    RemoteControlSet();

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
}