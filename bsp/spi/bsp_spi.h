#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "spi.h"

#define SPI_DEVICE_CNT 2
#define SPI_MX_REGISTER_CNT 4
// #define SPI_RXBUFF_LIMIT 64

typedef enum
{
    SPI_TRANSFER_BLOCKING_MODE = 0,
    SPI_TRANSFER_IT_MODE,
    SPI_TRANSFER_DMA_MODE
}SPI_Transfer_Mode_e;

typedef struct _SPIInstance
{
    SPI_HandleTypeDef *handle;
    GPIO_TypeDef *cs_GPIO;
    SPI_Transfer_Mode_e spi_work_mode;
    uint16_t cs_pin;
    uint16_t rx_size;
    uint8_t *rx_buffer;
    void(*spi_module_callback)(struct _SPIInstance *);
    void *id;
}SPIInstance;

typedef struct
{
    SPI_HandleTypeDef *handle;
    GPIO_TypeDef *cs_GPIO;
    uint16_t cs_pin;
    SPI_Transfer_Mode_e spi_work_mode;

    void(*spi_module_callback)(SPIInstance *);
    void *id;
}SPI_Init_Config_s;

SPIInstance *SPIRegister(SPI_Init_Config_s *config);
void SPITransmit(SPIInstance *instance, uint8_t *data, uint16_t size);
void SPIRecv(SPIInstance *instance, uint8_t *data, uint16_t size);
void SPITransRecv(SPIInstance *instance, uint8_t *data_tx, uint8_t *data_rx, uint16_t size);
void SPISetMode(SPIInstance *instance, SPI_Transfer_Mode_e spi_work_mode);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

#endif