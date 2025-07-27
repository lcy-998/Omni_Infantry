#include "bsp_can.h"
#include "can.h"
#include <string.h>
#include <stdlib.h>

static CANInstance *can_instance_list[CAN_MX_REGISTER_CNT];
static uint8_t idx;

static void CANAddFilter(CANInstance * instance)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_index = 0, can2_filter_index = 14;
    
    can_filter_conf.FilterIdLow = instance->rx_id << 5;
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_conf.SlaveStartFilterBank = 14;
    can_filter_conf.FilterFIFOAssignment = (instance->txconf.StdId & 1) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    can_filter_conf.FilterBank = instance->can_handle == &hcan1 ? (can1_filter_index++) : (can2_filter_index++); 
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    HAL_CAN_ConfigFilter(instance->can_handle, &can_filter_conf);
}

static void CANServiceInit(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}



CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if(!idx)
    {
        CANServiceInit();
    }

    //超出最大实例数警告
    //id重复警告

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    memset(instance, 0, sizeof(CANInstance));

    instance->txconf.StdId = config->tx_id;
    instance->txconf.IDE = CAN_ID_STD;
    instance->txconf.DLC = 0x08;
    instance->can_handle = config->can_handle;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    
    CANAddFilter(instance);
    can_instance_list[idx++] = instance;

    return instance;
}

uint8_t CANTransmit(CANInstance *instance)
{
    //邮箱等待超时警告

    HAL_CAN_AddTxMessage(instance->can_handle, &instance->txconf, instance->tx_buffer, &instance->tx_mailbox);

    //发送失败警告

    return 1;
}

void CANSetDLC(CANInstance *instance, uint8_t length)
{

    //检查是否出现野指针或越界访问

    instance->txconf.DLC = length;
}


static void CANFIFOxCallback(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buffer[8];
    while(HAL_CAN_GetRxFifoFillLevel(hcan, fifo))
    {
        HAL_CAN_GetRxMessage(hcan, fifo, &rxconf, can_rx_buffer);
        for(uint8_t i = 0; i < idx; i++)
        {
            if(hcan == can_instance_list[i]->can_handle && rxconf.StdId == can_instance_list[i]->rx_id)
            {
                can_instance_list[i]->rx_len = rxconf.DLC;
                memcpy(can_instance_list[i]->rx_buffer, can_rx_buffer, rxconf.DLC);
                if(can_instance_list[i]->can_module_callback != NULL)
                {
                    can_instance_list[i]->can_module_callback(can_instance_list[i]);
                }
                return;
            }
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}
