#include "remote_control.h"
#include "bsp_usart.h"
#include "daemon.h"

#define abs(x)  ((x) > 0 ? (x) : -(x))

static RC_Ctrl_t rc_ctrl[2];
static uint8_t rc_init_flag = 0;

static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

static void RectifyRCjoystick(void)
{
    for(size_t i = 0; i < 5; i++)
    {
        if(abs(*(&rc_ctrl[TEMP].rc.ch0 + i)) > 660)
        *(&rc_ctrl[TEMP].rc.ch0 + i) = 0;
    }
}


static void Sbus_to_RC(const uint8_t *sbus_buff)
{
    rc_ctrl[TEMP].rc.ch0 = ((sbus_buff[0] | (sbus_buff[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.ch1 = (((sbus_buff[1] >> 3) | (sbus_buff[2] << 5)) & 0x07ff) -RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.ch2 = (((sbus_buff[2] >> 6) | (sbus_buff[3] << 2) | (sbus_buff[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.ch3 = (((sbus_buff[4] >> 1) | (sbus_buff[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.dial = ((sbus_buff[16] | (sbus_buff[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    RectifyRCjoystick();
    rc_ctrl[TEMP].rc.s1 = ((sbus_buff[5] >> 4) & 0x0003);
    rc_ctrl[TEMP].rc.s2 = ((sbus_buff[5] >> 4) & 0x000C) >> 2;

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_Ctrl_t));
}

static void RemoteControlRxCallback(void)
{
    DaemonReload(rc_daemon_instance);
    Sbus_to_RC(rc_usart_instance->recv_buff);
}

static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(RC_Ctrl_t));
    USARTServiceInit(rc_usart_instance);
}

RC_Ctrl_t *RemoteControlInit(UART_HandleTypeDef *handle)
{
    USART_Init_Config_s uconfig;
    uconfig.usart_handle = handle;
    uconfig.usart_module_callback = RemoteControlRxCallback;
    uconfig.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    uconfig.usart_transfer_mode = USART_TRANSFER_NONE;
    rc_usart_instance = USARTRegister(&uconfig);

    Daemon_Init_Config_s dconfig = {
        .reload_count = 100,
        .offline_callback = RCLostCallback,
        .owner_id = NULL,
        .init_count = 100
    };
    rc_daemon_instance = DaemonRegister(&dconfig);
    
    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline(void)
{
    if(rc_init_flag)
    {
        return DaemonIsOnline(rc_daemon_instance);
    }
    return 0;
}