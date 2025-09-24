#include "bsp_spi.h"
#include "stdlib.h"
#include "memory.h"

static SPIInstance *spi_instance_list[SPI_MX_REGISTER_CNT] = {0};
static uint8_t idx = 0;

SPIInstance *SPIRegister(SPI_Init_Config_s *config)
{
    SPIInstance *instance = (SPIInstance *)malloc(sizeof(SPIInstance));
    memset(instance, 0, sizeof(SPIInstance));

    instance->handle = config->handle;
    instance->cs_GPIO = config->cs_GPIO;
    instance->cs_pin = config->cs_pin;
    instance->spi_module_callback = config->spi_module_callback;
    instance->id = config->id;
    instance->spi_work_mode = config->spi_work_mode;

    spi_instance_list[idx++] = instance;
    return instance;
}

void SPITransmit(SPIInstance *instance, uint8_t *data, uint16_t size)
{
    while(HAL_SPI_GetState(instance->handle) == HAL_SPI_STATE_BUSY);
    HAL_GPIO_WritePin(instance->cs_GPIO, instance->cs_pin, GPIO_PIN_RESET);
    switch (instance->spi_work_mode)
    {
        case SPI_TRANSFER_BLOCKING_MODE:
            HAL_SPI_Transmit(instance->handle, data, size, 1000);
            HAL_GPIO_WritePin(instance->cs_GPIO, instance->cs_pin, GPIO_PIN_SET);
            break;
        case SPI_TRANSFER_IT_MODE:
            HAL_SPI_Transmit_IT(instance->handle, data, size);
            break;
        case SPI_TRANSFER_DMA_MODE:
            HAL_SPI_Transmit_DMA(instance->handle, data, size);
            break;
    }
}

void SPIRecv(SPIInstance *instance, uint8_t *data, uint16_t size)
{
    instance->rx_buffer = data;
    instance->rx_size = size;
    while(HAL_SPI_GetState(instance->handle) == HAL_SPI_STATE_BUSY);
    HAL_GPIO_WritePin(instance->cs_GPIO, instance->cs_pin, GPIO_PIN_RESET);
    switch (instance->spi_work_mode)
    {
        case SPI_TRANSFER_BLOCKING_MODE:
            HAL_SPI_Receive(instance->handle, data, size, 1000);
            HAL_GPIO_WritePin(instance->cs_GPIO, instance->cs_pin, GPIO_PIN_SET);
            break;
        case SPI_TRANSFER_IT_MODE:
            HAL_SPI_Receive_IT(instance->handle, data, size);
            break;
        case SPI_TRANSFER_DMA_MODE:
            HAL_SPI_Receive_DMA(instance->handle, data, size);
            break;
    }
}

void SPITransRecv(SPIInstance *instance, uint8_t *data_tx, uint8_t *data_rx, uint16_t size)
{
    instance->rx_buffer = data_rx;
    instance->rx_size = size;
    while(HAL_SPI_GetState(instance->handle) == HAL_SPI_STATE_BUSY);//是否要加
    HAL_GPIO_WritePin(instance->cs_GPIO, instance->cs_pin, GPIO_PIN_RESET);
    switch (instance->spi_work_mode)
    {
        case SPI_TRANSFER_BLOCKING_MODE:
            HAL_SPI_TransmitReceive(instance->handle, data_tx, data_rx, size, 1000);
            HAL_GPIO_WritePin(instance->cs_GPIO, instance->cs_pin, GPIO_PIN_SET);
            break;
        case SPI_TRANSFER_IT_MODE:
            HAL_SPI_TransmitReceive_IT(instance->handle, data_tx, data_rx, size);
            break;
        case SPI_TRANSFER_DMA_MODE:
            HAL_SPI_TransmitReceive_DMA(instance->handle, data_tx, data_rx, size);
            break;
    }
}

void SPISetMode(SPIInstance *instance, SPI_Transfer_Mode_e spi_work_mode)
{
    if(instance->spi_work_mode != spi_work_mode)
    {
        instance->spi_work_mode = spi_work_mode;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for(size_t i = 0; i < idx; i++)
    {
        if(spi_instance_list[i]->handle == hspi && 
            HAL_GPIO_ReadPin(spi_instance_list[i]->cs_GPIO, spi_instance_list[i]->cs_pin) == GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(spi_instance_list[i]->cs_GPIO, spi_instance_list[i]->cs_pin, GPIO_PIN_SET);
            if(spi_instance_list[i]->spi_module_callback != NULL)
                spi_instance_list[i]->spi_module_callback(spi_instance_list[i]);
            return;
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_RxCpltCallback(hspi);
}


