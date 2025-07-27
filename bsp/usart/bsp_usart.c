#include "bsp_usart.h"
#include <memory.h>
#include <stdlib.h>

static uint8_t idx;
static USARTInstance *usart_instance_list[USART_MX_REGISER_CNT];

void USARTServiceInit(USARTInstance *instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle, instance->recv_buff, instance->recv_buff_size);
    __HAL_DMA_DISABLE_IT(instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *config)
{
    //是否超过最大实例数
    //是否重复注册

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->recv_buff_size = config->recv_buff_size;
    instance->usart_handle = config->usart_handle;
    instance->usart_module_callback = config->usart_module_callback;

    usart_instance_list[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

void USARTSend(USARTInstance *instance)
{
    switch(instance->usart_transfer_mode)
    {
        case USART_TRANSFER_BLOCKING:
            HAL_UART_Transmit(instance->usart_handle, instance->send_buff, instance->send_buff_size, 100);
            break;
        case USART_TRANSFER_IT:
            HAL_UART_Transmit_IT(instance->usart_handle, instance->send_buff, instance->send_buff_size);
            break;
        case USART_TRANSFER_DMA:
            HAL_UART_Transmit_DMA(instance->usart_handle, instance->send_buff, instance->send_buff_size);
            break;
        default: break;

    }
}

uint8_t USARTISReady(USARTInstance *instance)
{
    if(instance->usart_handle->gState == HAL_UART_STATE_BUSY_TX)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for(size_t i = 0; i < idx; i++)
    {
        if(huart == usart_instance_list[i]->usart_handle)
        {
            
            if(usart_instance_list[i]->usart_module_callback != NULL)
            {
                usart_instance_list[i]->usart_module_callback();
                
            }
            memset(usart_instance_list[i]->recv_buff, 0, USART_TXBUFF_LIMIT);
            HAL_UARTEx_ReceiveToIdle_DMA(huart, usart_instance_list[i]->recv_buff, usart_instance_list[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for(size_t i = 0; i < idx; i++)
    {
        if(huart == usart_instance_list[i]->usart_handle)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, usart_instance_list[i]->recv_buff, usart_instance_list[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

            return;
        }
    }
}
