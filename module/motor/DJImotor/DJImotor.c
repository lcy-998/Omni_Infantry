#include "DJImotor.h"
#include "memory.h"
#include "general_def.h"
#include "controller.h"
#include <stdlib.h>

static uint8_t idx = 0;
static DJIMotorInstance *dji_motor_insatance_list[DJI_MOTOR_CNT];

static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.DLC = 0x08, .txconf.RTR = CAN_RTR_DATA, .tx_buffer = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.DLC = 0x08, .txconf.RTR = CAN_RTR_DATA, .tx_buffer = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.DLC = 0x08, .txconf.RTR = CAN_RTR_DATA, .tx_buffer = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.DLC = 0x08, .txconf.RTR = CAN_RTR_DATA, .tx_buffer = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.DLC = 0x08, .txconf.RTR = CAN_RTR_DATA, .tx_buffer = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.DLC = 0x08, .txconf.RTR = CAN_RTR_DATA, .tx_buffer = {0}},
};

static uint8_t sender_enable_flag[6] = {0};

static void MotorSenderGrouping(DJIMotorInstance *instance, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1;
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (instance->motor_type)
    {
        case M2006:
        case M3508:
            if(motor_id < 4)
            {
                motor_send_num = motor_id;
                motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
            }
            else
            {
                motor_send_num = motor_id - 4;
                motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
            }
            config->rx_id = 0x200 + motor_id + 1;
            sender_enable_flag[motor_grouping] = 1;
            instance->sender_group = motor_grouping;
            instance->message_num = motor_send_num;

            //检查id是否发生冲突

            break;
        case GM6020 :
        case LK9025 :
        case HT04 : break;
        default: break;
    }
}

static void DecodeDJIMotor(CANInstance *instance)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)instance->id;
    DJI_Motor_Measures_s *measure = &motor->measure;
    uint8_t *rx_buffer = instance->rx_buffer;

    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rx_buffer[0]) << 8 | rx_buffer[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                        RPM_2_APS * SPEED_SMOOTH_COEF * (float)((int16_t)((rx_buffer[2]) << 8 | rx_buffer[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                         CURRENT_SMOOTH_COEF * (float)((int16_t)((rx_buffer[4]) << 8 | rx_buffer[5]));
    measure->temperature = rx_buffer[6];

    if((measure->ecd - measure->last_ecd) > 4096)
    {
        measure->total_round --;
    }
    else if((measure->ecd - measure->last_ecd) < -4096)
    {
        measure->total_round ++;
    }
    measure->total_angle = 360.0 * measure->total_round + measure->angle_single_round;
}


DJIMotorInstance *DJIMotorRegister(Motor_Init_Config_s *config)
{
    DJIMotorInstance *instance = (DJIMotorInstance*)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0 ,sizeof(DJIMotorInstance));

    instance->motor_type = config->motor_type; //Motor_Type_e
    instance->motor_setting = config->motor_setting;//Motor_Setting_s

    instance->motor_controller.current_pid = PIDRegister(&config->motor_controller_init.current_pid_init);
    instance->motor_controller.speed_pid = PIDRegister(&config->motor_controller_init.speed_pid_init);
    instance->motor_controller.angle_pid = PIDRegister(&config->motor_controller_init.angle_pid_init);
    instance->motor_controller.other_angle_feedback_ptr = config->motor_controller_init.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->motor_controller_init.other_speed_feedback_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->motor_controller_init.speed_feedforward_ptr;
    instance->motor_controller.current_feedforward_ptr = config->motor_controller_init.current_feedforward_ptr;//Motor_Controller_s

    MotorSenderGrouping(instance, &config->can_init_config);
    config->can_init_config.can_module_callback = DecodeDJIMotor;
    config->can_init_config.id = instance;
    instance->motor_can_instance = CANRegister(&config->can_init_config); //CANInstance

    DJIMotorEnable(instance);
    dji_motor_insatance_list[idx++] = instance;
    return instance;
}


void DJIMotorChangeFeed(DJIMotorInstance *instance, Closeloop_Type_e loop,Feedback_Sourse_e feedback_source)
{
    if(loop == ANGLE_LOOP)
    {
        instance->motor_setting.angle_feedback_source = feedback_source;
    }
    else if(loop == SPEED_LOOP)
    {
        instance->motor_setting.speed_feedback_source = feedback_source;
    }
}

void DJIMotorStop(DJIMotorInstance *instance)
{
    instance->stop_flag = MOTOR_STOP;
}


void DJIMotorEnable(DJIMotorInstance *instance)
{
    instance->stop_flag = MOTOR_ENABLE;
}

void DJIMotorSetLoop(DJIMotorInstance *instance, Closeloop_Type_e loop)
{
    instance->motor_setting.close_loop = loop;
}

void DJIMotorSetRef(DJIMotorInstance *instance, float ref)
{
    instance->motor_controller.pid_ref = ref;
}

void DJIMotorControl(void)
{
    uint8_t group, num;
    int16_t set;
    DJIMotorInstance *instance;
    Motor_Controller_s *motor_controller;
    Motor_Setting_s *motor_setting;
    DJI_Motor_Measures_s *dji_motor_measure;
    float pid_measure, pid_ref;

    for(size_t i = 0; i < idx; i++)
    {
        instance = dji_motor_insatance_list[i];
        motor_setting = &instance->motor_setting;
        motor_controller = &instance->motor_controller;
        dji_motor_measure = &instance->measure;
        pid_ref = motor_controller->pid_ref;
        
        if(instance->stop_flag != MOTOR_STOP)
        {
            if(motor_setting->close_loop & ANGLE_LOOP)
            {
                if(motor_setting->angle_feedback_source == OTHER_FEED)
                {
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                }
                else
                {
                    pid_measure = dji_motor_measure->total_angle;
                }
                pid_ref = PIDCalculate(motor_controller->angle_pid, pid_measure, pid_ref);
            }

            if(motor_setting->close_loop & SPEED_LOOP)
            {
                if(motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                {
                    pid_ref += *motor_controller->speed_feedforward_ptr;
                }
                if(motor_setting->speed_feedback_source == OTHER_FEED)
                {
                    pid_measure = *motor_controller->other_speed_feedback_ptr;
                }
                else
                {
                    pid_measure = dji_motor_measure->speed_aps;
                }
                pid_ref = PIDCalculate(motor_controller->speed_pid, pid_measure, pid_ref);
            }
            
            if(motor_setting->close_loop & CURRENT_LOOP)
            {
                pid_ref = PIDCalculate(motor_controller->current_pid, dji_motor_measure->real_current, pid_ref);
            }

            if(motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            {
                pid_ref += *motor_controller->current_feedforward_ptr;
            }

            set = (int16_t)pid_ref;
          
            group = instance->sender_group;
            num = instance->message_num;

            sender_assignment[group].tx_buffer[2 * num] = (uint8_t)(set >> 8);
            sender_assignment[group].tx_buffer[2 * num + 1] = (uint8_t)(set & 0x00ff);
        }
        else
        {
            group = instance->sender_group;
            num = instance->message_num;
            memset(&sender_assignment[group].tx_buffer[2 * num], 0, sizeof(uint16_t));
        }
    
    }
 
   

    for(size_t i = 0; i < 6; i++)
    {
        if(sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i]);
        }
    }
}