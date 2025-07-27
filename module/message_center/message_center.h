#ifndef __MESSAGE_CENTER_H__
#define __MESSAGE_CENTER_H__

#include <stdint.h>

#define MAX_TOPIC_NAME_LEN 32
#define MAX_TOPIC_COUNT 12
#define QUEUE_LEN 1

typedef struct _Subscriber_s
{
    void *queue[QUEUE_LEN];
    uint8_t data_len;
    uint8_t front_idx;
    uint8_t back_idx;
    uint8_t temp_size;

    struct _Subscriber_s *next_sub_queue;
}Subscriber_s;

typedef struct _Publisher_s
{
    char topic_name[MAX_TOPIC_NAME_LEN + 1];
    uint8_t data_len;
    Subscriber_s *first_subs;
    struct _Publisher_s *next_topic_node;
    uint8_t pub_registered_flag;
}Publisher_s;

Publisher_s *PubRegister(char *name, uint8_t data_len);
Subscriber_s *SubRegister(char *name, uint8_t data_len);
uint8_t SubGetMessage(Subscriber_s *sub, void *data_p);
uint8_t PubPushMessage(Publisher_s *pub, void *data_p);

#endif