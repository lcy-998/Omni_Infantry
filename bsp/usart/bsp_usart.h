#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "main.h"
#include "usart.h"

#define USART_MX_REGISER_CNT 3
#define USART_RXBUFF_LIMIT 256
#define USART_TXBUFF_LIMIT 256

typedef void (*USART_Module_Callback)(void);

typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA
}USART_Transfer_Mode_e;

typedef struct 
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT];
    uint16_t recv_buff_size;
    UART_HandleTypeDef *usart_handle;
    uint8_t send_buff[USART_TXBUFF_LIMIT];
    uint16_t send_buff_size;
    USART_Transfer_Mode_e usart_transfer_mode;
    USART_Module_Callback usart_module_callback;
}USARTInstance;

typedef struct 
{
    uint8_t recv_buff_size;
    UART_HandleTypeDef *usart_handle;
    uint8_t send_buff_size;
    USART_Transfer_Mode_e usart_transfer_mode;
    USART_Module_Callback usart_module_callback;
}USART_Init_Config_s;

USARTInstance *USARTRegister(USART_Init_Config_s *config);
void USARTServiceInit(USARTInstance *instance);
void USARTSend(USARTInstance *instance);
uint8_t USARTISReady(USARTInstance *instance);

#endif
