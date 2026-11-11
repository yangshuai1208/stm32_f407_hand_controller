#ifndef HAND_ACK_CACHE_H
#define HAND_ACK_CACHE_H

#include <stdint.h>

#include "hand_protocol.h"

#define ACK_CACHE_SIZE 4U

typedef enum
{
    ACK_STATUS_NONE = 0,
    ACK_STATUS_IN_PROGRESS,
    ACK_STATUS_OK,
    ACK_STATUS_PREEMPTED,
    ACK_STATUS_BUSY,
    ACK_STATUS_ERROR
} ack_status_t;

typedef struct
{
    uint8_t valid;
    uint32_t seq;
    hand_action_t action;
    ack_status_t status;
} hand_ack_record_t;

typedef struct
{
    hand_ack_record_t entries[ACK_CACHE_SIZE];
    uint8_t next;
} hand_ack_cache_t;

void hand_ack_cache_init(hand_ack_cache_t *cache);

hand_ack_record_t *hand_ack_cache_find(hand_ack_cache_t *cache,
                                       uint32_t seq);

void hand_ack_cache_set(hand_ack_cache_t *cache,
                        uint32_t seq,
                        hand_action_t action,
                        ack_status_t status);

#endif
