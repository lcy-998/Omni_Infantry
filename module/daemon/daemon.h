#ifndef __DAEMON_H__
#define __DAEMON_H__

#include <stdint.h>
#include <string.h>

#define DAEMON_MX_REGISTER_CNT 64

typedef void(*Offline_Callback)(void *);

typedef struct 
{
    uint16_t reload_count;
    Offline_Callback offline_callbak;
    uint16_t temp_count;
    void *owner_id;
}DaemonInstance;

typedef struct 
{
    uint16_t reload_count;
    uint16_t init_count;
    Offline_Callback offline_callback;
    void *owner_id;
}Daemon_Init_Config_s;

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(DaemonInstance *instance);
void DaemonTask(void);


#endif
