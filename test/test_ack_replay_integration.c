#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "hand_command_dispatch.h"
#include "hand_servo.h"
#include "hal_mock_transactions.h"

void hal_mock_reset(void);
void hal_mock_advance_tick(uint32_t milliseconds);
uint32_t hal_mock_get_i2c_write_count(void);

typedef struct
{
    uint32_t seq;
    ack_status_t status;
} ack_event_t;

static I2C_HandleTypeDef mock_i2c;
static hand_ack_cache_t ack_cache;
static hand_comm_safety_t comm_safety;
static uint8_t action_in_progress;
static uint32_t active_seq;
static hand_action_t active_action;
static ack_event_t ack_events[16U];
static size_t ack_event_count;
static uint32_t start_call_count;
static uint32_t conflict_count;

static HAL_StatusTypeDef counted_real_start(hand_action_t action)
{
    start_call_count++;
    return hand_servo_start_action(action);
}

static void record_ack(uint32_t seq, ack_status_t status)
{
    assert(ack_event_count < sizeof(ack_events) / sizeof(ack_events[0]));
    ack_events[ack_event_count].seq = seq;
    ack_events[ack_event_count].status = status;
    ack_event_count++;
}

static void record_conflict(void)
{
    conflict_count++;
}

static const hand_command_dispatch_ops_t dispatch_ops =
{
    counted_real_start,
    hand_servo_is_busy,
    record_ack,
    record_conflict
};

static void dispatch(uint32_t seq, hand_action_t action)
{
    hand_command_t command = {seq, action};
    hand_command_dispatch(&command, &action_in_progress, &active_seq,
                          &active_action, &ack_cache, &comm_safety,
                          &dispatch_ops);
}

static void service_motion(void)
{
    hand_command_service_motion(&action_in_progress, &active_seq,
                                &active_action, &ack_cache,
                                hand_servo_update, hand_servo_is_busy,
                                record_ack);
}

static void complete_motion(uint32_t seq, hand_action_t action)
{
    for (uint32_t step = 0U; step < 20U && action_in_progress != 0U; step++)
    {
        hal_mock_advance_tick(20U);
        service_motion();
    }
    assert(action_in_progress == 0U);
    assert(active_seq == 0U);
    assert(active_action == HAND_ACTION_NONE);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    hand_ack_record_t *record = hand_ack_cache_find(&ack_cache, seq);
    assert(record != NULL);
    assert(record->action == action);
    assert(record->status == ACK_STATUS_OK);
}

static void expect_ack(size_t index, uint32_t seq, ack_status_t status)
{
    assert(index < ack_event_count);
    assert(ack_events[index].seq == seq);
    assert(ack_events[index].status == status);
}

int main(void)
{
    assert(ACK_CACHE_SIZE == 4U);
    hal_mock_reset();
    hand_ack_cache_init(&ack_cache);
    hand_comm_safety_init(&comm_safety);
    assert(hand_servo_init(&mock_i2c) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);

    dispatch(100U, HAND_ACTION_GRAB);
    assert(start_call_count == 1U);
    assert(action_in_progress == 1U);
    assert(active_seq == 100U);
    assert(active_action == HAND_ACTION_GRAB);
    assert(ack_cache.next == 1U);
    expect_ack(0U, 100U, ACK_STATUS_IN_PROGRESS);
    complete_motion(100U, HAND_ACTION_GRAB);
    expect_ack(1U, 100U, ACK_STATUS_OK);
    assert(ack_cache.next == 1U); /* Status update is in place. */

    /* Move away from GRAB so a later replay can produce observable PWM writes. */
    dispatch(200U, HAND_ACTION_OPEN);
    assert(start_call_count == 2U);
    expect_ack(2U, 200U, ACK_STATUS_IN_PROGRESS);
    complete_motion(200U, HAND_ACTION_OPEN);
    expect_ack(3U, 200U, ACK_STATUS_OK);
    assert(ack_cache.next == 2U);

    dispatch(201U, HAND_ACTION_STOP);
    dispatch(202U, HAND_ACTION_STOP);
    assert(start_call_count == 4U);
    assert(ack_cache.next == 0U);
    assert(hand_ack_cache_find(&ack_cache, 100U) != NULL);
    assert(hand_ack_cache_find(&ack_cache, 100U)->status == ACK_STATUS_OK);
    expect_ack(4U, 201U, ACK_STATUS_OK);
    expect_ack(5U, 202U, ACK_STATUS_OK);

    /* Retransmission while cached returns the terminal ACK without motion. */
    uint32_t writes_before_cached = hal_mock_get_i2c_write_count();
    dispatch(100U, HAND_ACTION_GRAB);
    service_motion();
    assert(start_call_count == 4U);
    assert(ack_cache.next == 0U);
    assert(action_in_progress == 0U);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(hal_mock_get_i2c_write_count() == writes_before_cached);
    expect_ack(6U, 100U, ACK_STATUS_OK);

    /* A new sequence writes the actual next slot, evicting the old GRAB. */
    dispatch(203U, HAND_ACTION_STOP);
    assert(start_call_count == 5U);
    assert(ack_cache.next == 1U);
    assert(hand_ack_cache_find(&ack_cache, 100U) == NULL);
    assert(hand_ack_cache_find(&ack_cache, 203U)->status == ACK_STATUS_OK);
    expect_ack(7U, 203U, ACK_STATUS_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before_cached);

    hal_mock_i2c_transactions_reset();
    uint32_t writes_before_replay = hal_mock_get_i2c_write_count();
    dispatch(100U, HAND_ACTION_GRAB);
    assert(start_call_count == 6U);
    assert(action_in_progress == 1U);
    assert(active_seq == 100U);
    assert(active_action == HAND_ACTION_GRAB);
    assert(hand_servo_get_state() == HAND_MOTION_GRABBING);
    assert(ack_cache.next == 2U);
    assert(hand_ack_cache_find(&ack_cache, 100U)->status ==
           ACK_STATUS_IN_PROGRESS);
    assert(hal_mock_get_i2c_write_count() == writes_before_replay);
    expect_ack(8U, 100U, ACK_STATUS_IN_PROGRESS);

    hal_mock_advance_tick(20U);
    service_motion();
    assert(hal_mock_get_i2c_write_count() ==
           writes_before_replay + HAND_SERVO_NUM);
    assert(hal_mock_i2c_transaction_count() == HAND_SERVO_NUM);
    for (size_t channel = 0U; channel < HAND_SERVO_NUM; channel++)
    {
        const hal_mock_i2c_transaction_t *write =
            hal_mock_i2c_transaction_at(channel);
        assert(write != NULL);
        assert(write->device_address == 0x80U);
        assert(write->register_address == 0x06U + 4U * channel);
        assert(write->data_length == 4U);
        assert(write->result == HAL_OK);
    }
    assert(action_in_progress == 1U);
    assert(hand_ack_cache_find(&ack_cache, 100U)->status ==
           ACK_STATUS_IN_PROGRESS);
    assert(ack_event_count == 9U);

    complete_motion(100U, HAND_ACTION_GRAB);
    assert(hal_mock_get_i2c_write_count() >
           writes_before_replay + HAND_SERVO_NUM);
    assert(ack_event_count == 10U);
    expect_ack(9U, 100U, ACK_STATUS_OK);
    assert(conflict_count == 0U);
    assert(hal_mock_i2c_transaction_overflow_count() == 0U);

    puts("ack_replay_integration: cached retry=OK without motion; "
         "evicted replay=IN_PROGRESS with motion, then OK");
    return 0;
}
