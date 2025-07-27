#include "message_center.h"
#include <string.h>
#include <stdlib.h>

static Publisher_s message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL
};

Publisher_s *PubRegister(char *name, uint8_t data_len)
{
    Publisher_s *node = &message_center;
    while(node->next_topic_node)
    {
        node = node->next_topic_node;
        if(strcmp(node->topic_name, name) == 0)
        {
            node->pub_registered_flag = 1;
            return node;
        }
    }

    node->next_topic_node = (Publisher_s *)malloc(sizeof(Publisher_s));
    memset(node->next_topic_node, 0, sizeof(Publisher_s));

    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->pub_registered_flag = 1;
    return node->next_topic_node;
}

Subscriber_s *SubRegister(char *name, uint8_t data_len)
{
    Publisher_s *pub = PubRegister(name, data_len);
    Subscriber_s *sub = (Subscriber_s *)malloc(sizeof(Subscriber_s));
    memset(sub, 0, sizeof(Subscriber_s));

    sub->data_len = data_len;
    for(size_t i = 0; i < QUEUE_LEN; i++)
    {
        sub->queue[i] = malloc(data_len);
    }

    if(pub->first_subs == NULL)
    {
        pub->first_subs = sub;
        return sub;
    }

    Subscriber_s *node = pub->first_subs;
    while(node->next_sub_queue)
    {
        node = node->next_sub_queue;
    }
    node->next_sub_queue = sub;
    return sub;
}

uint8_t SubGetMessage(Subscriber_s *sub, void *data_p)
{
    if(sub->temp_size == 0)
    {
        return 0;
    }

    memcpy(data_p, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx + 1) % QUEUE_LEN;
    sub->temp_size--;
    return 1;
}

uint8_t PubPushMessage(Publisher_s *pub, void *data_p)
{
    static Subscriber_s *sub;
    sub = pub->first_subs;

    while(sub)
    {
        if(sub->temp_size == QUEUE_LEN)
        {
            sub->front_idx = (sub->front_idx + 1) % QUEUE_LEN;
            sub->temp_size --;
        }

        memcpy(sub->queue[sub->back_idx], data_p, pub->data_len);
        sub->back_idx = (sub->back_idx + 1) % QUEUE_LEN;
        sub->temp_size ++;

        sub = sub->next_sub_queue;
    }
    return 1;
}


