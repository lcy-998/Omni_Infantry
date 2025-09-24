// #include "kalman_filter.h"
// #include "stdlib.h"

// /**
//  * @brief Initializes the matrix instance.
//  * 
//  * @param _MAT_ matrix instance. 
//  * @param _ROW_ the row of matrix.
//  * @param _COLUMN_ the column of matrix.
//  * @param _TYPE_ the datatype of the element of matrix.
//  * 
//  * @retval None
//  */

// #define MATRIX_INIT(_MAT_, _ROW_, _COLUMN_, _TYPE_)                                           \
//     {                                                                                   \
//         _MAT_.pData = (_TYPE_ *)malloc(sizeof(_TYPE_) * (_ROW_) * (_COLUMN_));                   \
//         memset(_MAT_.pData, 0, sizeof(_TYPE_) * (_ROW_) * (_COLUMN_));            \
//         _MAT_.numRows = (_ROW_); \
//         _MAT_.numCols = (_COLUMN_); \
//     } \

// static void SwapDataptr(float **pa, float **pb)
// {
//     float *temp;
//     temp = *pa;
//     *pa = *pb;
//     *pb = temp;
// }

// static void H_K_R_Adjustment(KalmanFilter_t *kf)
// {
//     kf->MeasurementValidNum = 0;
//     memcpy(kf->z.pData, kf->MeasuredVector, sizeof(float) * kf->zSize);
//     memset(kf->MeasuredVector, 0, sizeof(float) * kf->zSize);

//     memset(kf->R.pData, 0, sizeof(float) * kf->zSize * kf->zSize);
//     memset(kf->H.pData, 0, sizeof(float) * kf->xhatSize * kf->zSize);

//     for(size_t i = 0; i < kf->zSize; i++)
//     {
//         if(kf->z.pData[i] != 0)
//         {
//             kf->z.pData[kf->MeasurementValidNum] = kf->z.pData[i];
//             kf->temp[kf->MeasurementValidNum] = i;

//             kf->H.pData[kf->H.numCols * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];

//             kf->MeasurementValidNum++;
//         }
//     }
//     for(size_t i = 0; i < kf->MeasurementValidNum; i++)
//     {
//         kf->R.pData[i * kf->MeasurementValidNum + i] = kf->MatR_DiagomalElements[kf->temp[i]];
//     }
//     kf->H.numRows = kf->MeasurementValidNum;
//     kf->H.numCols = kf->xhatSize;
//     kf->HT.numCols = kf->xhatSize;
//     kf->HT.numRows = kf->MeasurementValidNum;
//     kf->R.numRows = kf->MeasurementValidNum;
//     kf->R.numCols = kf->MeasurementValidNum;
//     kf->K.numRows = kf->xhatSize;
//     kf->K.numCols = kf->MeasurementValidNum;
//     kf->z.numRows = kf->MeasurementValidNum;
// }
    

// void KalmanFilter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
// {
//     kf->xhatSize = xhatSize;
//     kf->uSize = uSize;
//     kf->zSize = zSize;

//     kf->FilteredValue = (float *)malloc(sizeof(float) * xhatSize);
//     kf->MeasuredVector = (float *)malloc(sizeof(float) * zSize);
//     kf->ControlVector = (float *)malloc(sizeof(float) * uSize);

//     kf->MeasurementValidNum = 0;
//     kf->MeasurementMap = (uint8_t *)malloc(sizeof(uint8_t) * zSize);
//     memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
//     kf->temp = (uint8_t *)malloc(sizeof(uint8_t) * zSize);
//     memset(kf->temp, 0, sizeof(uint8_t) * zSize);
//     kf->MeasurementDegree = (float *)malloc(sizeof(float) * zSize);
//     memset(kf->MeasurementDegree, 0, sizeof(float) * zSize);
//     kf->MatR_DiagomalElements = (float *)malloc(sizeof(float) * zSize);
//     memset(kf->MatR_DiagomalElements, 0, sizeof(float) * zSize);

//     MATRIX_INIT(kf->xhat, xhatSize, 1, float);
//     MATRIX_INIT(kf->xhatminus, xhatSize, 1, float);

//     if(uSize != 0)
//     {
//         MATRIX_INIT(kf->u, uSize, 1, float);
//     }

//     MATRIX_INIT(kf->z, zSize, 1, float);
//     MATRIX_INIT(kf->P, xhatSize, xhatSize, float);
//     MATRIX_INIT(kf->Pminus, xhatSize, xhatSize, float);
//     MATRIX_INIT(kf->F, xhatSize, xhatSize, float);
//     MATRIX_INIT(kf->FT, xhatSize, xhatSize, float);

//     if(uSize != 0)
//     {
//         MATRIX_INIT(kf->B, xhatSize, uSize, float);
//     }

//     MATRIX_INIT(kf->H, zSize, xhatSize, float);
//     MATRIX_INIT(kf->HT, xhatSize, zSize, float);
//     MATRIX_INIT(kf->Q, xhatSize, xhatSize, float);
//     MATRIX_INIT(kf->R, zSize, zSize, float);

//     MATRIX_INIT(kf->K, xhatSize, zSize, float);

//     MATRIX_INIT(kf->temp_mat1, xhatSize, xhatSize, float);
//     MATRIX_INIT(kf->temp_mat2, xhatSize, xhatSize, float);
//     MATRIX_INIT(kf->temp_vector1, xhatSize, 1, float);
//     MATRIX_INIT(kf->temp_vector2, xhatSize, 1, float);

//     kf->SkipEq1 = 0;
//     kf->SkipEq2 = 0;
//     kf->SkipEq3 = 0;
//     kf->SkipEq4 = 0;
//     kf->SkipEq5 = 0;
// }

// void KalmanFilter_Measure(KalmanFilter_t *kf)
// {
//     if(kf->UseAutoAdjustment != 0)
//     {
//         H_K_R_Adjustment(kf);
//     }
//     else
//     {
//         SwapDataptr(&kf->z.pData, &kf->MeasuredVector);
//         memset(kf->MeasuredVector, 0, sizeof(float) * kf->zSize);
//     }
//     SwapDataptr(&kf->u.pData, &kf->ControlVector);
// }

// void KalmanFilter_xhatMinusUpdate(KalmanFilter_t *kf)
// {
//     if(!kf->SkipEq1)
//     {
//         if(kf->uSize > 0)
//         {
//             kf->mat_status = arm_mat_mult_f32(&kf->F, &kf->xhat, &kf->temp_vector1);
//             kf->mat_status = arm_mat_mult_f32(&kf->B, &kf->u, &kf->temp_vector2);
//             kf->mat_status = arm_mat_add_f32(&kf->temp_vector1, &kf->temp_vector2, &kf->xhatminus);
//         }
//         else
//         {
//             kf->mat_status = arm_mat_mult_f32(&kf->F, &kf->xhat, &kf->xhatminus);
//         }//x' = Fx + Bu
//     }
//     else
//     {
//         if(kf->user_func[1] != NULL)
//         {
//             kf->user_func[1](kf);
//         }
//     }
    
// }

// void KalmanFilter_PminusUpdate(KalmanFilter_t *kf)
// {
//     if(!kf->SkipEq2)
//     {
//         kf->mat_status = arm_mat_trans_f32(&kf->F, &kf->FT);
//         kf->mat_status = arm_mat_mult_f32(&kf->F, &kf->P, &kf->Pminus);
//         kf->mat_status = arm_mat_mult_f32(&kf->Pminus, &kf->FT, &kf->temp_mat1);
//         kf->mat_status = arm_mat_add_f32(&kf->Q, &kf->temp_mat1, &kf->Pminus);
//     }//P' = F P FT + Q
//     else
//     {
//         if(kf->user_func[2] != NULL)
//         {
//             kf->user_func[2](kf);
//         }
//     }
// }

// void KalmanFilter_SetK(KalmanFilter_t *kf)
// {
//     if(!kf->SkipEq3)
//     {
//         kf->mat_status = arm_mat_trans_f32(&kf->H, &kf->HT);
//         kf->mat_status = arm_mat_mult_f32(&kf->H, &kf->Pminus, &kf->temp_mat1);
//         kf->mat_status = arm_mat_mult_f32(&kf->temp_mat1, &kf->HT, &kf->temp_mat2);
//         kf->mat_status = arm_mat_add_f32(&kf->temp_mat2, &kf->R, &kf->temp_mat1);
//         kf->mat_status = arm_mat_inverse_f32(&kf->temp_mat1, &kf->temp_mat2);
//         kf->mat_status = arm_mat_mult_f32(&kf->Pminus, &kf->HT, &kf->temp_mat1);
//         kf->mat_status = arm_mat_mult_f32(&kf->temp_mat1, &kf->temp_mat2, &kf->K);
//     }// K = P' HT / (H P' HT + R)
//     else
//     {
//         if(kf->user_func[3] != NULL)
//         {
//             kf->user_func[3](kf);
//         }
//     }
// }

// void KalmanFilter_xhatUpdate(KalmanFilter_t *kf)
// {
//     if(!kf->SkipEq4)
//     {
//         kf->mat_status = arm_mat_mult_f32(&kf->H, &kf->xhatminus, &kf->temp_vector1);
//         kf->mat_status = arm_mat_sub_f32(&kf->z, &kf->temp_vector1, &kf->temp_vector2);
//         kf->mat_status = arm_mat_mult_f32(&kf->K, &kf->temp_vector2, &kf->temp_vector1);
//         kf->mat_status = arm_mat_add_f32(&kf->temp_vector1, &kf->xhatminus, &kf->xhat);
//     }// x = K (H x' -z) + x'
//     else
//     {
//         if(kf->user_func[4] != NULL)
//         {
//             kf->user_func[4](kf);
//         }
//     }
// }

// void KalmanFilter_PUpdate(KalmanFilter_t *kf)
// {
//     if(!kf->SkipEq5)
//     {
//         kf->mat_status = arm_mat_mult_f32(&kf->K, &kf->H, &kf->temp_mat1);
//         kf->mat_status = arm_mat_mult_f32(&kf->temp_mat1, &kf->Pminus, &kf->temp_mat1);
//         kf->mat_status = arm_mat_sub_f32(&kf->P, &kf->temp_mat1, &kf->P);
//     }
//     else
//     {
//         if(kf->user_func[5] != NULL)
//         {
//             kf->user_func[5](kf);
//         }
//     }
// }

// float *KalmanFilter_Update(KalmanFilter_t *kf)
// {
//     KalmanFilter_Measure(kf);
    
//     KalmanFilter_xhatMinusUpdate(kf);
//     KalmanFilter_PminusUpdate(kf);
//     if(kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
//     {
//         KalmanFilter_SetK(kf);
//         KalmanFilter_xhatUpdate(kf);
//         KalmanFilter_PUpdate(kf);
//     }
//     else
//     {
        
//     }
    
//     memcpy(kf->FilteredValue, &kf->xhat.pData, sizeof(float) * kf->xhatSize);
//     return kf->FilteredValue;
// }

/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of kalman filter
 ******************************************************************************
 * @attention
 * 该卡尔曼滤波器可以在传感器采样频率不同的情况下，动态调整矩阵H R和K的维数与数值。
 * This implementation of kalman filter can dynamically adjust dimension and
 * value of matrix H R and K according to the measurement validity under any
 * circumstance that the sampling rate of component sensors are different.
 *
 * 因此矩阵H和R的初始化会与矩阵P A和Q有所不同。另外的，在初始化量测向量z时需要额外写
 * 入传感器量测所对应的状态与这个量测的方式，详情请见例程
 * Therefore, the initialization of matrix P, F, and Q is sometimes different
 * from that of matrices H R. when initialization. Additionally, the corresponding
 * state and the method of the measurement should be provided when initializing
 * measurement vector z. For more details, please see the example.
 *
 * 若不需要动态调整量测向量z，可简单将结构体中的Use_Auto_Adjustment初始化为0，并像初
 * 始化矩阵P那样用常规方式初始化z H R即可。
 * If automatic adjustment is not required, assign zero to the UseAutoAdjustment
 * and initialize z H R in the normal way as matrix P.
 *
 * 要求量测向量z与控制向量u在传感器回调函数中更新。整数0意味着量测无效，即自上次卡尔曼
 * 滤波更新后无传感器数据更新。因此量测向量z与控制向量u会在卡尔曼滤波更新过程中被清零
 * MeasuredVector and ControlVector are required to be updated in the sensor
 * callback function. Integer 0 in measurement vector z indicates the invalidity
 * of current measurement, so MeasuredVector and ControlVector will be reset
 * (to 0) during each update.
 *
 * 此外，矩阵P过度收敛后滤波器将难以再适应状态的缓慢变化，从而产生滤波估计偏差。该算法
 * 通过限制矩阵P最小值的方法，可有效抑制滤波器的过度收敛，详情请见例程。
 * Additionally, the excessive convergence of matrix P will make filter incapable
 * of adopting the slowly changing state. This implementation can effectively
 * suppress filter excessive convergence through boundary limiting for matrix P.
 * For more details, please see the example.
 *
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // 设置最小方差
 *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // 开启自动调整
 *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // 设置矩阵值
 *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // 循环更新
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // 测量数据更新应按照以下形式 即向MeasuredVector赋值
 * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */

#include "kalman_filter.h"

uint16_t sizeof_float, sizeof_double;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
 * @brief 初始化矩阵维度信息并为矩阵分配空间
 *
 * @param kf kf类型定义
 * @param xhatSize 状态变量维度
 * @param uSize 控制变量维度
 * @param zSize 观测量维度
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    // measurement flags
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // filter data
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // xhat x(k|k)
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    // xhatminus x(k|k-1)
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0)
    {
        // control vector u
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    // measurement vector z
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    // covariance matrix P(k|k)
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    // create covariance matrix P(k|k-1)
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    // state transition matrix F FT
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0)
    {
        // control matrix B
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

    // measurement matrix H
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    // process noise covariance matrix Q
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    // measurement noise covariance matrix R
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    // kalman gain K
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 矩阵H K R根据量测情况自动调整
    // matrix H K R auto adjustment
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)·H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
 * @brief 执行卡尔曼滤波黄金五式,提供了用户定义函数,可以替代五个中的任意一个环节,方便自行扩展为EKF/UKF/ESKF/AUKF等
 * 
 * @param kf kf类型定义
 * @return float* 返回滤波值
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. 获取量测信息
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 先验估计
    // 1. xhat'(k)= A·xhat(k-1) + B·u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 预测更新
    // 2. P'(k) = A·P(k-1)·AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 量测更新
        // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 融合
        // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 修正方差
        // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // 无有效量测,仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // 自定义函数,可以提供后处理等
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 避免滤波器过度收敛
    // suppress filter excessive convergence
    for (uint8_t i = 0; i < kf->xhatSize; i++)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // 识别量测数据有效性并调整矩阵H R K
    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; i++)
    {
        if (kf->z_data[i] != 0)
        {
            // 重构向量z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // 重构矩阵H
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++)
    {
        // 重构矩阵R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // 调整矩阵维数
    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
