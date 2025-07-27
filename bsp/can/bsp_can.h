#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "main.h"
#include "can.h"

#define CAN_MX_REGISTER_CNT 8

typedef struct _CANInstance
{
    CAN_HandleTypeDef *can_handle;
    CAN_TxHeaderTypeDef txconf;
    uint32_t tx_mailbox;
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[8];
    uint32_t rx_id;
    uint8_t rx_len;

    void (*can_module_callback)(struct _CANInstance *);
    void *id;
}CANInstance;

typedef struct 
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
}CAN_Init_Config_s;

CANInstance *CANRegister(CAN_Init_Config_s *config);
uint8_t CANTransmit(CANInstance *instance);
void CANSetDLC(CANInstance *instance, uint8_t length);

#endif
