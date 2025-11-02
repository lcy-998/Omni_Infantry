#include "BMI088.h"
#include "BMI088Reg.h"
#include "bsp_spi.h"
#include "memory.h"
#include "bsp_dwt.h"
#include "stdlib.h"

static void BMI088AccRead(BMI088Instance *instance, uint8_t reg, uint8_t *data, uint8_t len)
{
    static uint8_t tx_data[8];
    static uint8_t rx_data[8];
    tx_data[0] = 0x80 | reg;
    SPITransRecv(instance->spi_acc, tx_data, rx_data, len + 2);
    memcpy(data, rx_data + 2,  len);
}

static void BMI088GyroRead(BMI088Instance *instance, uint8_t reg, uint8_t *data, uint8_t len)
{
    static uint8_t tx_data[7];
    static uint8_t rx_data[7];
    tx_data[0] = 0x80 | reg;
    SPITransRecv(instance->spi_gyro, tx_data, rx_data, len + 1);
    memcpy(data, rx_data + 1, len);
}

static void BMI088AccWriteSingleReg(BMI088Instance *instance, uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg, data};
    SPITransmit(instance->spi_acc, tx_data, 2);
}

static void BMI088GyroWriteSingleReg(BMI088Instance *instance, uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg, data};
    SPITransmit(instance->spi_gyro, tx_data, 2);
}

/*----------------------------------------------------------------------------*/
#define BMI088REG 0
#define BMI088DATA 1
#define BMI088ERROR 2
static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};
// BMI088初始化配置数组for gyro,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};


static uint8_t BMI088AccInit(BMI088Instance *instance)
{
    uint8_t ACC_Chip_ID = 0;

    BMI088AccRead(instance, BMI088_ACC_CHIP_ID, &ACC_Chip_ID, 1);
    DWT_Delay(0.001f);

    BMI088AccWriteSingleReg(instance, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    DWT_Delay(0.001f);

    BMI088AccRead(instance, BMI088_ACC_CHIP_ID, &ACC_Chip_ID, 1);
    DWT_Delay(0.001f);

    BMI088AccRead(instance, BMI088_ACC_CHIP_ID, &ACC_Chip_ID, 1);
    if(ACC_Chip_ID != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001f);

    uint8_t reg = 0, data = 0;
    BMI088_ERROR_CODE_e error = BMI088_NO_ERROR;
    for(size_t i = 0; i < sizeof(BMI088_Accel_Init_Table) / sizeof(BMI088_Accel_Init_Table[0]); i++)
    {
        reg = BMI088_Accel_Init_Table[i][BMI088REG];
        data = BMI088_Accel_Init_Table[i][BMI088DATA];
        BMI088AccWriteSingleReg(instance, reg, data);
        DWT_Delay(0.001f);
        BMI088AccRead(instance, reg, &data, 1);
        DWT_Delay(0.001f);
        if(data != BMI088_Accel_Init_Table[i][BMI088DATA])
            error |= BMI088_Accel_Init_Table[i][BMI088ERROR];
    }

    switch (BMI088_Accel_Init_Table[3][1])
    {
        case BMI088_ACC_RANGE_3G:
            instance->BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
            break;
        case BMI088_ACC_RANGE_6G:
            instance->BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
            break;
        case BMI088_ACC_RANGE_12G:
            instance->BMI088_ACCEL_SEN = BMI088_ACCEL_12G_SEN;
            break;
        case BMI088_ACC_RANGE_24G:
            instance->BMI088_ACCEL_SEN = BMI088_ACCEL_24G_SEN;
            break;
    }
    return error;
}

static uint8_t BMI088GyroInit(BMI088Instance *instance)
{
    BMI088GyroWriteSingleReg(instance, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    DWT_Delay(0.030F);

    uint8_t GYRO_Chip_ID = 0;
    BMI088GyroRead(instance, BMI088_GYRO_CHIP_ID, &GYRO_Chip_ID, 1);
    if(GYRO_Chip_ID != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001F);

    uint8_t reg = 0, data = 0;
    BMI088_ERROR_CODE_e error = BMI088_NO_ERROR;
    for(size_t i = 0; i < sizeof(BMI088_Gyro_Init_Table) / sizeof(BMI088_Gyro_Init_Table[0]); i++)
    {
        reg  = BMI088_Gyro_Init_Table[i][BMI088REG];
        data = BMI088_Gyro_Init_Table[i][BMI088DATA];
        BMI088GyroWriteSingleReg(instance, reg, data); 
        DWT_Delay(0.001f);
        BMI088GyroRead(instance, reg, &data, 1);
        DWT_Delay(0.001f);
        if(data != BMI088_Gyro_Init_Table[i][BMI088DATA])
            error |= BMI088_Gyro_Init_Table[i][BMI088ERROR];
    }

    switch (BMI088_Gyro_Init_Table[0][1])
    {
        case BMI088_GYRO_2000:
            instance->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
            break;
        case BMI088_GYRO_1000:
            instance->BMI088_GYRO_SEN = BMI088_GYRO_1000_SEN;
            break;
        case BMI088_GYRO_500:
            instance->BMI088_GYRO_SEN = BMI088_GYRO_500_SEN;
            break;
        case BMI088_GYRO_250:
            instance->BMI088_GYRO_SEN = BMI088_GYRO_250_SEN;
            break;
        case BMI088_GYRO_125:
            instance->BMI088_GYRO_SEN = BMI088_GYRO_125_SEN;
            break;
    }
    instance->gyro_offset[0] = 0.00193356676;
    instance->gyro_offset[1] = -0.00593106402f;
    instance->gyro_offset[2] = -0.0002371210995f;
    return error;
}

void BMI088TempControl(BMI088Instance *instance)
{

}

/*----------------------------------------------------------------------------*/

static void BMI088AccSPIFinishCallback(SPIInstance *spi)
{
    
}

static void BMI088GyroSPIFinishCallback(SPIInstance *spi)
{

}

static void BMI088AccINTCallback(GPIOInstance *gpio)
{

}

static void BMI088GyroINTCallback(GPIOInstance *gpio)
{

}

/*----------------------------------------------------------------------------*/

static void BMI088SetMode(BMI088Instance *instance, BMI088_Work_Mode_e mode)
{
    instance->bmi088_work_mode = mode;
    if(mode == BMI088_BLOCK_PERIODIC_MDOE)
    {
        SPISetMode(instance->spi_acc, SPI_TRANSFER_BLOCKING_MODE);
        SPISetMode(instance->spi_gyro, SPI_TRANSFER_BLOCKING_MODE);
    }
    else if (mode == BMI088_BLOCK_TRIGGER_MODE)
    {
        SPISetMode(instance->spi_acc, SPI_TRANSFER_DMA_MODE);
        SPISetMode(instance->spi_gyro, SPI_TRANSFER_DMA_MODE);
    }
}

/*----------------------------------------------------------------------------*/

uint8_t BMI088Acquire(BMI088Instance *instance)
{
    static uint8_t bmi088_temp;
    BMI088_Data_s *data = &instance->bmi088_data;
    if(instance->bmi088_work_mode == BMI088_BLOCK_PERIODIC_MDOE)
    {
        static uint8_t buf[6] = {0};
        BMI088AccRead(instance, BMI088_ACCEL_XOUT_L, buf, 6);
        for(size_t i = 0; i < 3; i++)
        {
            data->acc[i] = instance->BMI088_ACCEL_SEN * (float)(int16_t)((buf[2 * i + 1] << 8) | buf[2 * i]);
        }
        BMI088GyroRead(instance, BMI088_GYRO_X_L, buf, 6);
        for(size_t i = 0; i < 3; i++)
        {
            data->gyro[i] = instance->BMI088_GYRO_SEN * (float)(int16_t)((buf[2 * i + 1] << 8) | buf[2 * i]);
        }
        BMI088AccRead(instance, BMI088_TEMP_M, buf, 2);
        bmi088_temp = (float)(uint16_t)((buf[1] >> 5) | buf[0] << 3);
        if(bmi088_temp > 1023)
        {
            bmi088_temp -= 2048;
        }//why?
        data->temperature = bmi088_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        return 1;
    }
}

uint8_t BMI088Acquire_IT_Status(BMI088Instance *instance)
{

}

void BMI088CalibrateIMU(BMI088Instance *instance)
{
    // if(instance->bmi088_cali_mode == BMI088_CALIBRATE_ONLINE_MODE)
    // {
    //     instance->acc_coef = BMI088_ACCEL_6G_SEN;
    //     instance->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
    //     float startTime;
    //     uint16_t CaliTimes = 6000;

    // }

}

BMI088Instance *BMI088Register(BMI088_Init_Config_s *config)
{
    BMI088Instance *instance = (BMI088Instance *)malloc(sizeof(BMI088Instance));
    config->spi_acc_init_config.id =
        config->spi_gyro_init_config.id =
            config->gpio_acc_init_config.id =
                config->gpio_gyro_init_config.id =
                    config->heat_pwm_init_config.id = instance;

    if(config->bmi088_work_mode == BMI088_BLOCK_PERIODIC_MDOE)
    {
        config->spi_acc_init_config.spi_work_mode = SPI_TRANSFER_BLOCKING_MODE;
        config->spi_gyro_init_config.spi_work_mode = SPI_TRANSFER_BLOCKING_MODE;
    }
    else if(config->bmi088_work_mode == BMI088_BLOCK_TRIGGER_MODE)
    {
        config->spi_acc_init_config.spi_work_mode = SPI_TRANSFER_DMA_MODE;
        config->spi_gyro_init_config.spi_work_mode = SPI_TRANSFER_DMA_MODE;

        config->spi_acc_init_config.spi_module_callback  = BMI088AccSPIFinishCallback;
        config->spi_gyro_init_config.spi_module_callback = BMI088GyroSPIFinishCallback;
        config->gpio_acc_init_config.gpio_module_callback = BMI088AccINTCallback;
        config->gpio_gyro_init_config.gpio_module_callback = BMI088GyroINTCallback;

        instance->acc_int = GPIORegister(&config->gpio_acc_init_config);
        instance->gyro_int = GPIORegister(&config->gpio_gyro_init_config);
    }

    instance->spi_acc = SPIRegister(&config->spi_acc_init_config);
    instance->spi_gyro = SPIRegister(&config->spi_gyro_init_config);
    instance->heat_pwm = PWMRegister(&config->heat_pwm_init_config);
    instance->heat_pid = PIDRegister(&config->heat_pid_init_config);

    BMI088SetMode(instance, BMI088_BLOCK_PERIODIC_MDOE);
    BMI088_ERROR_CODE_e error = BMI088_NO_ERROR;
    do {
        error = BMI088_NO_ERROR;
        error |= BMI088AccInit(instance);
        error |= BMI088GyroInit(instance);
    }while (error != BMI088_NO_ERROR);
    instance->bmi088_cali_mode = config->bmi088_cali_mode;
    BMI088CalibrateIMU(instance);
    BMI088SetMode(instance, config->bmi088_work_mode);

    return instance;
}
