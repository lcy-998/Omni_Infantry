
#include "robot_def.h"
#include "DJImotor.h"
#include "message_center.h"

#include "math.h"

#define _1_SQRT2 0.7071067f

#define MOTOR_LF_ID 1
#define MOTOR_RF_ID 2
#define MOTOR_LB_ID 4
#define MOTOR_RB_ID 3

static Publisher_s *chassis_pub;
static Subscriber_s *chassis_sub;

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback_data;

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;

static float wt_lf, wt_rf, wt_lb, wt_rb; //轮子角速度max = 54000
float temp_v;

void ChassisInit(void)
{
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .motor_controller_init = {
            .speed_pid_init = {
                .Kp = 4.5,
                .Ki = 0,
                .Kd = 0,
                .IntegralLimit = 3000,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3,
            },
            
        },
        .motor_setting = {
            .close_loop = SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
        .motor_type = M3508,
    };

    chassis_motor_config.can_init_config.tx_id = MOTOR_LF_ID;
    chassis_motor_config.motor_setting.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorRegister(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = MOTOR_RF_ID;
    chassis_motor_config.motor_setting.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = DJIMotorRegister(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = MOTOR_LB_ID;
    chassis_motor_config.motor_setting.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorRegister(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = MOTOR_RB_ID;
    chassis_motor_config.motor_setting.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorRegister(&chassis_motor_config);

    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
}

static void WheelSpeedCalculate(void)
{
    wt_lf = -(_1_SQRT2 * chassis_cmd_recv.vx / WHEEL_RADIUS) + (_1_SQRT2 * chassis_cmd_recv.vy / WHEEL_RADIUS) + (chassis_cmd_recv.wz * HALF_TRACK_WIDTH / WHEEL_RADIUS);
    wt_lb = -(_1_SQRT2 * chassis_cmd_recv.vx / WHEEL_RADIUS) - (_1_SQRT2 * chassis_cmd_recv.vy / WHEEL_RADIUS) + (chassis_cmd_recv.wz * HALF_TRACK_WIDTH / WHEEL_RADIUS);
    wt_rb = (_1_SQRT2 * chassis_cmd_recv.vx / WHEEL_RADIUS) - (_1_SQRT2 * chassis_cmd_recv.vy / WHEEL_RADIUS) + (chassis_cmd_recv.wz * HALF_TRACK_WIDTH / WHEEL_RADIUS);
    wt_rf = (_1_SQRT2 * chassis_cmd_recv.vx / WHEEL_RADIUS) + (_1_SQRT2 * chassis_cmd_recv.vy / WHEEL_RADIUS) + (chassis_cmd_recv.wz * HALF_TRACK_WIDTH / WHEEL_RADIUS); //轮子角速度

    wt_lf *= M3508_RATIO;
    wt_lb *= M3508_RATIO;
    wt_rb *= M3508_RATIO;
    wt_rf *= M3508_RATIO;  //电机角速度
}

static void LimitChassisOutput(void)
{
    
    DJIMotorSetRef(motor_lf, wt_lf);
    DJIMotorSetRef(motor_rf, wt_rf);
    DJIMotorSetRef(motor_lb, wt_lb);
    DJIMotorSetRef(motor_rb, wt_rb);
}

static void ChassisSpeedCalculate(void)
{
    wt_lf = motor_lf->measure.speed_aps / M3508_RATIO;
    wt_rf = motor_rf->measure.speed_aps / M3508_RATIO;
    wt_lb = motor_lb->measure.speed_aps / M3508_RATIO;
    wt_rb = motor_rb->measure.speed_aps / M3508_RATIO;

    chassis_feedback_data.vx = -(WHEEL_RADIUS * _1_SQRT2 / 2.0 / wt_lf) - (WHEEL_RADIUS * _1_SQRT2 / 2.0 / wt_lb) + (WHEEL_RADIUS * _1_SQRT2 / 2.0 / wt_rb) + (WHEEL_RADIUS * _1_SQRT2 / 2.0 / wt_rf);
    chassis_feedback_data.vy =  (WHEEL_RADIUS * _1_SQRT2 / 2.0 / wt_lf) - (WHEEL_RADIUS * _1_SQRT2 / 2.0 / wt_lb) - (WHEEL_RADIUS * _1_SQRT2 / 2.0 / wt_rb) + (WHEEL_RADIUS * _1_SQRT2 / 2.0 / wt_rf);
    chassis_feedback_data.wz =  (WHEEL_RADIUS / 4.0 / HALF_TRACK_WIDTH) * (wt_lb + wt_lf + wt_rb + wt_rf);
}

void ChassisTask(void)
{
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    if(chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    {
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_ROTATE:
        chassis_cmd_recv.wz = CHASSIS_APS_MX;
        break;
    
    case CHASSIS_FOLLOW:
        break;    
    
    case CHASSIS_NO_FOLLOW:
        break;
    }
    
    WheelSpeedCalculate();

    LimitChassisOutput();

    ChassisSpeedCalculate();

    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}