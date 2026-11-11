#include "hand_ack_cache.h"

void hand_ack_cache_init(hand_ack_cache_t *cache)
{
    if (cache == 0)
    {
        return;
    }

    for (uint8_t i = 0U; i < ACK_CACHE_SIZE; i++)
    {
        cache->entries[i].valid = 0U;
        cache->entries[i].seq = 0U;
        cache->entries[i].action = HAND_ACTION_NONE;
        cache->entries[i].status = ACK_STATUS_NONE;
    }

    cache->next = 0U;
}

hand_ack_record_t *hand_ack_cache_find(hand_ack_cache_t *cache,
                                       uint32_t seq)
{
    if (cache == 0)
    {
        return 0;
    }

    for (uint8_t i = 0U; i < ACK_CACHE_SIZE; i++)
    {
        if (cache->entries[i].valid != 0U &&
            cache->entries[i].seq == seq)
        {
            return &cache->entries[i];
        }
    }

    return 0;
}

void hand_ack_cache_set(hand_ack_cache_t *cache,
                        uint32_t seq,
                        hand_action_t action,
                        ack_status_t status)
{
    if (cache == 0)
    {
        return;
    }

    hand_ack_record_t *record = hand_ack_cache_find(cache, seq);
    if (record != 0)
    {
        record->status = status;
        return;
    }

    cache->entries[cache->next].valid = 1U;
    cache->entries[cache->next].seq = seq;
    cache->entries[cache->next].action = action;
    cache->entries[cache->next].status = status;

    cache->next = (uint8_t)((cache->next + 1U) % ACK_CACHE_SIZE);
}
