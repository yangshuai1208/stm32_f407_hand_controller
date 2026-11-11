#include <assert.h>
#include <stdio.h>

#include "hand_ack_cache.h"

static hand_ack_cache_t cache;

static void reset_cache(void)
{
    hand_ack_cache_init(&cache);
}

static void test_init_and_empty_cache(void)
{
    reset_cache();

    assert(cache.next == 0U);
    for (uint8_t i = 0U; i < ACK_CACHE_SIZE; i++)
    {
        assert(cache.entries[i].valid == 0U);
        assert(cache.entries[i].status == ACK_STATUS_NONE);
    }
    assert(hand_ack_cache_find(&cache, 1U) == 0);
}

static void test_first_insert_and_lookup(void)
{
    reset_cache();
    hand_ack_cache_set(&cache, 11U, HAND_ACTION_GRAB,
                       ACK_STATUS_IN_PROGRESS);

    hand_ack_record_t *record = hand_ack_cache_find(&cache, 11U);
    assert(record != 0);
    assert(record->valid == 1U);
    assert(record->seq == 11U);
    assert(record->action == HAND_ACTION_GRAB);
    assert(record->status == ACK_STATUS_IN_PROGRESS);
}

static void test_same_seq_updates_status_and_repeat_lookup(void)
{
    reset_cache();
    hand_ack_cache_set(&cache, 12U, HAND_ACTION_OPEN,
                       ACK_STATUS_IN_PROGRESS);
    hand_ack_cache_set(&cache, 12U, HAND_ACTION_OPEN,
                       ACK_STATUS_OK);

    hand_ack_record_t *first = hand_ack_cache_find(&cache, 12U);
    hand_ack_record_t *second = hand_ack_cache_find(&cache, 12U);
    assert(first != 0);
    assert(first == second);
    assert(first->action == HAND_ACTION_OPEN);
    assert(first->status == ACK_STATUS_OK);
}

static void test_same_seq_different_action_preserves_original_action(void)
{
    reset_cache();
    hand_ack_cache_set(&cache, 13U, HAND_ACTION_GRAB,
                       ACK_STATUS_PREEMPTED);
    hand_ack_cache_set(&cache, 13U, HAND_ACTION_STOP,
                       ACK_STATUS_OK);

    hand_ack_record_t *record = hand_ack_cache_find(&cache, 13U);
    assert(record != 0);
    /* Existing cache semantics update status only for an existing seq. */
    assert(record->action == HAND_ACTION_GRAB);
    assert(record->status == ACK_STATUS_OK);
}

static void test_four_entries_and_fifth_insert_evicts_oldest(void)
{
    reset_cache();
    for (uint32_t seq = 1U; seq <= ACK_CACHE_SIZE; seq++)
    {
        hand_ack_cache_set(&cache, seq, HAND_ACTION_OPEN,
                           ACK_STATUS_OK);
    }

    for (uint32_t seq = 1U; seq <= ACK_CACHE_SIZE; seq++)
    {
        assert(hand_ack_cache_find(&cache, seq) != 0);
    }

    hand_ack_cache_set(&cache, 5U, HAND_ACTION_GRAB,
                       ACK_STATUS_IN_PROGRESS);
    assert(hand_ack_cache_find(&cache, 1U) == 0);
    assert(hand_ack_cache_find(&cache, 2U) != 0);
    assert(hand_ack_cache_find(&cache, 5U) != 0);
}

static void test_evicted_seq_is_inserted_as_new_record_on_reappearance(void)
{
    reset_cache();
    for (uint32_t seq = 20U; seq <= 24U; seq++)
    {
        hand_ack_cache_set(&cache, seq, HAND_ACTION_GRAB,
                           ACK_STATUS_OK);
    }

    assert(hand_ack_cache_find(&cache, 20U) == 0);
    hand_ack_cache_set(&cache, 20U, HAND_ACTION_GRAB,
                       ACK_STATUS_IN_PROGRESS);

    hand_ack_record_t *record = hand_ack_cache_find(&cache, 20U);
    assert(record != 0);
    assert(record->action == HAND_ACTION_GRAB);
    assert(record->status == ACK_STATUS_IN_PROGRESS);
}

static void test_replacement_index_wraps(void)
{
    reset_cache();
    for (uint32_t seq = 30U; seq <= 38U; seq++)
    {
        hand_ack_cache_set(&cache, seq, HAND_ACTION_RELEASE,
                           ACK_STATUS_OK);
    }

    /* Four slots have been replaced more than twice; the index wrapped. */
    for (uint32_t seq = 30U; seq <= 34U; seq++)
    {
        assert(hand_ack_cache_find(&cache, seq) == 0);
    }
    for (uint32_t seq = 35U; seq <= 38U; seq++)
    {
        assert(hand_ack_cache_find(&cache, seq) != 0);
    }
}

int main(void)
{
    test_init_and_empty_cache();
    test_first_insert_and_lookup();
    test_same_seq_updates_status_and_repeat_lookup();
    test_same_seq_different_action_preserves_original_action();
    test_four_entries_and_fifth_insert_evicts_oldest();
    test_evicted_seq_is_inserted_as_new_record_on_reappearance();
    test_replacement_index_wraps();

    puts("hand_ack_cache: all tests passed");
    return 0;
}
