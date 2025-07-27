#include "daemon.h"
#include "stdlib.h"

static uint8_t idx = 0;
static DaemonInstance *daemon_instance_list[DAEMON_MX_REGISTER_CNT] = {0};

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count;
    instance->offline_callbak = config->offline_callback;
    instance->temp_count = config->init_count == 0 ? 100 : config->init_count;

    daemon_instance_list[idx++] = instance;
    return instance;
}

void DaemonReload(DaemonInstance *instance)
{
    instance->temp_count = instance->reload_count;
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return instance->temp_count > 0;
}

void DaemonTask(void)
{
    DaemonInstance *instance;
    for(size_t i = 0; i < idx; i++)
    {
        instance = daemon_instance_list[i];
        if(instance->temp_count > 0)
        {
            instance->temp_count --;
        }
        else if(instance->offline_callbak)
        {
            instance->offline_callbak(instance->owner_id);
        }
    }
}