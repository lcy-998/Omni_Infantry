#include "robot_def.h"
#include "DJImotor.h"
#include "general_def.h"
#include "message_center.h"
#include "BMI088.h"
#include "ins_task.h"

static Publisher_s *gimbal_pub;
static Subscriber_s *gimbal_sub;

static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;
static Gimbal_Upload_Data_s gimbal_feedback_data;

static DJIMotorInstance *yaw_motor, *pitch_motor;

static BMI088Instance *bmi088;

extern INS_t INS;

void GimbalInit(void)
{
    BMI088_Init_Config_s bmi088_config = {
        .gpio_acc_init_config = {.gpio = GPIOC, .gpio_pin = GPIO_PIN_4},
        .gpio_gyro_init_config = {.gpio = GPIOC, .gpio_pin = GPIO_PIN_5},
        .spi_acc_init_config = {.cs_GPIO = GPIOA, .cs_pin = GPIO_PIN_4, .handle = &hspi1},
        .spi_gyro_init_config = {.cs_GPIO = GPIOB, .cs_pin = GPIO_PIN_0, .handle = &hspi1},
        .heat_pid_init_config = {
            .Kp = 10,
            .Ki = 2,
            .Kd = 0,
            .Improve = PID_IMPROVEMENT_NONE,
            .IntegralLimit = 0.15,
            .MaxOut = 1,},
        .heat_pwm_init_config = {
            .channel = TIM_CHANNEL_1,
            .handle = &htim10,
            .dutyratio = 0,
            .period = 0.001,
        },
        .bmi088_cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
        .bmi088_work_mode = BMI088_BLOCK_PERIODIC_MDOE,
    };
    bmi088 = BMI088Register(&bmi088_config);

    //以下电机初始化参数需要修改
    Motor_Init_Config_s yaw_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 5,
        },

        .motor_controller_init = {
            .current_pid_init = {
                .Kp = 0,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 16384,
            },
            .speed_pid_init = {
                .Kp = 180,
                .Ki = 10,
                .Kd = 0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Integral | PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16384,
                .Derivative_LPF_RC = 0,
            },
            .angle_pid_init = {
                .Kp = 0.8,
                .Ki = 0.01,
                .Kd = 0.0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Integral,
                .MaxOut = 100,
                .DeadBand = 0,
                .Derivative_LPF_RC = 0.27,
            },
            .other_speed_feedback_ptr = &INS.Gyro[2],
            .other_angle_feedback_ptr = &INS.YawTotalAngle,
        },
        .motor_setting = {
            .close_loop = SPEED_LOOP | ANGLE_LOOP,
            .speed_feedback_source = OTHER_FEED,
            .angle_feedback_source = OTHER_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = GM6020,
    };

    Motor_Init_Config_s pitch_motor_config = {

    };

    INS_Init(bmi088);

    yaw_motor = DJIMotorRegister(&yaw_motor_config);
    pitch_motor = DJIMotorRegister(&pitch_motor_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}


void GimbalTask(void)
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    case GIMBAL_FREE_MODE:
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    }

    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);

    gimbal_feedback_data.yaw = INS.Yaw;
    gimbal_feedback_data.yaw_total_angle = INS.YawTotalAngle;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}
