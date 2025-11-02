#include "bsp_dwt.h"
#include "stdbool.h"
#include "DJImotor.h"
#include "message_center.h"
#include "shoot.h"
#include "robot_def.h"

static DJIMotorInstance *loader, *friction_l, *friction_r;

static Publisher_s *shoot_pub;
static Subscriber_s *shoot_sub;

static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
static Shoot_Upload_Data_s shoot_feedback_data;

void ShootInit(void)
{
    Motor_Init_Config_s friction_config = {

    };
    friction_config.can_init_config.tx_id = 2;
    friction_config.motor_setting.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    //friction_l = DJIMotorRegister(&friction_config);

    friction_config.can_init_config.tx_id = 3;
    friction_config.motor_setting.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    //friction_r = DJIMotorRegister(&friction_config);

    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .motor_controller_init = {
            .speed_pid_init = {
                .Kp = 3.5,
                .Ki = 0.9,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
        },
        .motor_setting = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .close_loop = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedforward_flag = CURRENT_FEEDFORWARD,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = M2006
    };
    //loader = DJIMotorRegister(&loader_config);

    shoot_cmd_recv.shoot_mode = SHOOT_OFF;

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    DJIMotorStop(loader);
}

static float block_time;
static float reverse_time;
static float current_record[5];
static float current_record_average;
static float block_status;

static void Load_Reverse(void)
{
    for(size_t i = 0; i < 4; i++)
    {
        current_record[i] = current_record[i + 1];
    }
    current_record[4] = loader->measure.real_current;
    current_record_average = (current_record[0] + current_record[1] + current_record[2] + current_record[3] + current_record[4]) / 5.0f;

    if (current_record_average > BLOCK_CURRENT)
    {
        block_time++;
    }

    if(reverse_time >= 1)
    {
        shoot_cmd_recv.load_mode = LOAD_REVERSE;
        reverse_time ++;

        if(reverse_time >= 200)
        {
            reverse_time = 0;
            block_time = 0;
        }
    }

    if(loader->measure.real_current < REVERSE_BLOCK_CURRENT)
    {
        reverse_time = 0;
        block_time = 0;
    }
    else
    {
        if(block_time > 200)
        {
            reverse_time = 1;
        }
    }
}

static void Shoot_Fric_data_process(void)
{
    /*------------------------------------constant variance-------------------------------------*/
    static bool bullet_waiting_confirm = false;
    float data = friction_l->measure.speed_aps;
    static uint16_t data_history[MAX_HISTORY];
    static uint8_t head = 0, rear = 0;
    float moving_average[2];
    uint8_t data_num;
    float derivative;
    /*------------------------------------------------------------------------------------------*/

    data = my_abs(data);
    data_history[head] = data;
    head = (head + 1) % MAX_HISTORY;
    data_num = (head - rear + MAX_HISTORY) % MAX_HISTORY;
    if(data_num >= FILTER_WINDOWSIZE + 1)
    {
        moving_average[0] = 0;
        moving_average[1] = 0;

        for(uint8_t i = rear, j = rear + 1, index = rear; index < rear + FILTER_WINDOWSIZE; i++, j++, index++)
        {
            i %= MAX_HISTORY;
            j %= MAX_HISTORY;
            moving_average[0] += data_history[i];
            moving_average[1] += data_history[j];

            
        }
    }
}

void ShootTask(void)
{
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    if(shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    PubPushMessage(shoot_pub, &shoot_feedback_data);
}
