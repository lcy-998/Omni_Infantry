#ifndef __BMI088_H__
#define __BMI088_H__

#include "bsp_spi.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "controller.h"

typedef enum
{
    BMI088_BLOCK_PERIODIC_MDOE = 0,
    BMI088_BLOCK_TRIGGER_MODE
}BMI088_Work_Mode_e;

typedef enum
{
    BMI088_CALIBRATE_ONLINE_MODE = 0,
    BMI088_LOAD_PRE_CALI_MODE
}BMI088_Calibrate_Mode_e;

typedef struct 
{
    float gyro[3];
    float acc[3];
    float temperature;

}BMI088_Data_s;

typedef struct 
{
    BMI088_Work_Mode_e bmi088_work_mode;
    BMI088_Calibrate_Mode_e bmi088_cali_mode;
    SPIInstance *spi_gyro;
    SPIInstance *spi_acc;
    GPIOInstance *gyro_int;
    GPIOInstance *acc_int;
    PIDInstance *heat_pid;
    PWMInstance *heat_pwm;

    BMI088_Data_s bmi088_data;

    float gyro_offset[3];
    float gNorm;
    float acc_coef;

    float BMI088_ACCEL_SEN;
    float BMI088_GYRO_SEN;

}BMI088Instance;

typedef struct 
{
    BMI088_Work_Mode_e bmi088_work_mode;
    BMI088_Calibrate_Mode_e bmi088_cali_mode;
    SPI_Init_Config_s spi_gyro_init_config;
    SPI_Init_Config_s spi_acc_init_config;
    GPIO_Init_Config_s gpio_gyro_init_config;
    GPIO_Init_Config_s gpio_acc_init_config;
    PID_Init_Config_s heat_pid_init_config;
    PWM_Init_Config_s heat_pwm_init_config;

}BMI088_Init_Config_s;

uint8_t BMI088Acquire(BMI088Instance *instance);
BMI088Instance *BMI088Register(BMI088_Init_Config_s *config);


#endif
