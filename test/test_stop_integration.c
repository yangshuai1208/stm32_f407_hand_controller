#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "hand_command_dispatch.h"
#include "hand_servo.h"

void hal_mock_reset(void);
void hal_mock_advance_tick(uint32_t milliseconds);
void hal_mock_fail_next_i2c_write(void);
uint32_t hal_mock_get_i2c_write_count(void);

typedef struct
{
    uint32_t seq;
    ack_status_t status;
} ack_event_t;

#define ACK_EVENT_CAPACITY 8U

static I2C_HandleTypeDef mock_i2c;
static hand_ack_cache_t ack_cache;
static hand_comm_safety_t comm_safety;
static uint8_t action_in_progress;
static uint32_t active_seq;
static hand_action_t active_action;
static ack_event_t ack_events[ACK_EVENT_CAPACITY];
static size_t ack_event_count;
static uint32_t start_action_call_count;
static uint32_t stop_call_count;
static uint32_t seq_conflict_count;

static HAL_StatusTypeDef counted_real_start_action(hand_action_t action)
{
    start_action_call_count++;
    if (action == HAND_ACTION_STOP)
    {
        stop_call_count++;
    }

    return hand_servo_start_action(action);
}

static void record_ack(uint32_t seq, ack_status_t status)
{
    assert(ack_event_count < ACK_EVENT_CAPACITY);
    ack_events[ack_event_count].seq = seq;
    ack_events[ack_event_count].status = status;
    ack_event_count++;
}

static void record_seq_conflict(void)
{
    seq_conflict_count++;
}

static const hand_command_dispatch_ops_t dispatch_ops =
{
    counted_real_start_action,
    hand_servo_is_busy,
    record_ack,
    record_seq_conflict
};

static void reset_ack_observation(void)
{
    ack_event_count = 0U;
    seq_conflict_count = 0U;
}

static void reset_fixture(void)
{
    hal_mock_reset();
    hand_ack_cache_init(&ack_cache);
    hand_comm_safety_init(&comm_safety);
    action_in_progress = 0U;
    active_seq = 0U;
    active_action = HAND_ACTION_NONE;
    start_action_call_count = 0U;
    stop_call_count = 0U;
    reset_ack_observation();
    assert(hand_servo_init(&mock_i2c) == HAL_OK);
}

static void dispatch(uint32_t seq, hand_action_t action)
{
    hand_command_t command =
    {
        seq,
        action
    };

    hand_command_dispatch(&command,
                          &action_in_progress,
                          &active_seq,
                          &active_action,
                          &ack_cache,
                          &comm_safety,
                          &dispatch_ops);
}

static void expect_ack(size_t index,
                       uint32_t seq,
                       ack_status_t status)
{
    assert(index < ack_event_count);
    assert(ack_events[index].seq == seq);
    assert(ack_events[index].status == status);
}

static void test_grab_is_preempted_by_stop_with_original_ack_order(void)
{
    reset_fixture();

    dispatch(101U, HAND_ACTION_GRAB);
    assert(action_in_progress == 1U);
    assert(active_seq == 101U);
    assert(active_action == HAND_ACTION_GRAB);
    assert(hand_servo_get_state() == HAND_MOTION_GRABBING);
    assert(start_action_call_count == 1U);

    hal_mock_advance_tick(20U);
    assert(hand_servo_update() == HAL_OK);
    uint32_t writes_before_stop = hal_mock_get_i2c_write_count();

    dispatch(102U, HAND_ACTION_STOP);

    assert(stop_call_count == 1U);
    assert(start_action_call_count == 2U);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(hand_servo_is_busy() == 0U);
    assert(hal_mock_get_i2c_write_count() == writes_before_stop);
    assert(action_in_progress == 0U);
    assert(active_seq == 0U);
    assert(active_action == HAND_ACTION_NONE);

    hand_ack_record_t *grab = hand_ack_cache_find(&ack_cache, 101U);
    hand_ack_record_t *stop = hand_ack_cache_find(&ack_cache, 102U);
    assert(grab != NULL);
    assert(stop != NULL);
    assert(grab->action == HAND_ACTION_GRAB);
    assert(grab->status == ACK_STATUS_PREEMPTED);
    assert(stop->action == HAND_ACTION_STOP);
    assert(stop->status == ACK_STATUS_OK);

    assert(ack_event_count == 3U);
    expect_ack(0U, 101U, ACK_STATUS_IN_PROGRESS);
    expect_ack(1U, 101U, ACK_STATUS_PREEMPTED);
    expect_ack(2U, 102U, ACK_STATUS_OK);
    assert(seq_conflict_count == 0U);
}

static void test_duplicate_stop_uses_cached_ok_without_second_side_effect(void)
{
    reset_fixture();
    uint32_t writes_before_stop = hal_mock_get_i2c_write_count();

    dispatch(201U, HAND_ACTION_STOP);
    dispatch(201U, HAND_ACTION_STOP);

    assert(start_action_call_count == 1U);
    assert(stop_call_count == 1U);
    assert(hal_mock_get_i2c_write_count() == writes_before_stop);
    assert(action_in_progress == 0U);
    assert(active_seq == 0U);
    assert(active_action == HAND_ACTION_NONE);

    hand_ack_record_t *stop = hand_ack_cache_find(&ack_cache, 201U);
    assert(stop != NULL);
    assert(stop->action == HAND_ACTION_STOP);
    assert(stop->status == ACK_STATUS_OK);

    assert(ack_event_count == 2U);
    expect_ack(0U, 201U, ACK_STATUS_OK);
    expect_ack(1U, 201U, ACK_STATUS_OK);
    assert(seq_conflict_count == 0U);
}

static void test_active_same_seq_different_action_remains_conflict(void)
{
    reset_fixture();

    dispatch(401U, HAND_ACTION_GRAB);
    dispatch(401U, HAND_ACTION_STOP);

    assert(seq_conflict_count == 1U);
    assert(start_action_call_count == 1U);
    assert(stop_call_count == 0U);
    assert(action_in_progress == 1U);
    assert(active_seq == 401U);
    assert(active_action == HAND_ACTION_GRAB);
    assert(hand_servo_get_state() == HAND_MOTION_GRABBING);

    dispatch(402U, HAND_ACTION_STOP);
    assert(stop_call_count == 1U);
    assert(action_in_progress == 0U);
}

static void test_stop_in_fault_preserves_fault_and_existing_error(void)
{
    reset_fixture();

    dispatch(301U, HAND_ACTION_GRAB);
    hal_mock_advance_tick(20U);
    hal_mock_fail_next_i2c_write();
    assert(hand_servo_update() == HAL_ERROR);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);

    /* Establish the state produced by main.c's existing update-error tail. */
    hand_ack_cache_set(&ack_cache,
                       active_seq,
                       active_action,
                       ACK_STATUS_ERROR);
    action_in_progress = 0U;
    active_seq = 0U;
    active_action = HAND_ACTION_NONE;
    reset_ack_observation();

    uint32_t writes_before_stop = hal_mock_get_i2c_write_count();
    dispatch(302U, HAND_ACTION_STOP);

    assert(stop_call_count == 1U);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_servo_is_busy() == 0U);
    assert(hal_mock_get_i2c_write_count() == writes_before_stop);

    hand_ack_record_t *failed_grab =
        hand_ack_cache_find(&ack_cache, 301U);
    hand_ack_record_t *stop = hand_ack_cache_find(&ack_cache, 302U);
    assert(failed_grab != NULL);
    assert(stop != NULL);
    assert(failed_grab->action == HAND_ACTION_GRAB);
    assert(failed_grab->status == ACK_STATUS_ERROR);
    assert(stop->action == HAND_ACTION_STOP);
    assert(stop->status == ACK_STATUS_OK);

    assert(ack_event_count == 1U);
    expect_ack(0U, 302U, ACK_STATUS_OK);
    assert(seq_conflict_count == 0U);
}

int main(void)
{
    test_grab_is_preempted_by_stop_with_original_ack_order();
    test_duplicate_stop_uses_cached_ok_without_second_side_effect();
    test_active_same_seq_different_action_remains_conflict();
    test_stop_in_fault_preserves_fault_and_existing_error();

    puts("stop_integration: all tests passed");
    return 0;
}
