// #ifndef __QUAT_EKF_H
// #define __QUAT_EKF_H

// #include "kalman_filter.h"

// typedef struct 
// {
//     uint8_t initialized;
//     KalmanFilter_t IMU_QuaternionEKF;
//     uint64_t UpdateCount;
//     uint8_t StableFlag;

//     float q[4];
//     float GyroBias[3];

//     float Gyro[3];
//     float Acc[3];

//     float OrientationCosine[3];

//     float accLPFcoef;
//     float gyro_norm;
//     float accl_norm;

//     float Pitch;
//     float Roll;
//     float Yaw;

//     float YawLastAngle;
//     int32_t YawRoundCount;
//     float YawTotalAngle;
    
//     float Q1, Q2;
//     float R;

//     float dt;
//     arm_matrix_instance_f32 ChiSquare;
//     float ChiSquareTestThreshold;
//     float ChiSquare_Data[1];
//     float lambda;
// }QEKF_INS_t;


// void IMU_QuaternionEKF_Init(float *init_quaternion, float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
// void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

// #endif
/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif

