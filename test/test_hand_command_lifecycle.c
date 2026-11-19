#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "hand_command_dispatch.h"
#include "hand_servo.h"

void hal_mock_reset(void);
void hal_mock_advance_tick(uint32_t milliseconds);
void hal_mock_fail_next_i2c_write(void);

typedef struct
{
    uint32_t seq;
    ack_status_t status;
} ack_event_t;

#define ACK_EVENT_CAPACITY 16U

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

static void reset_fixture(void)
{
    hal_mock_reset();
    hand_ack_cache_init(&ack_cache);
    hand_comm_safety_init(&comm_safety);
    action_in_progress = 0U;
    active_seq = 0U;
    active_action = HAND_ACTION_NONE;
    ack_event_count = 0U;
    start_action_call_count = 0U;
    stop_call_count = 0U;
    seq_conflict_count = 0U;
    assert(hand_servo_get_state() != HAND_MOTION_FAULT);
    assert(hand_servo_init(&mock_i2c) == HAL_OK);
}

static void dispatch(uint32_t seq, hand_action_t action)
{
    hand_command_t command = {seq, action};
    hand_command_dispatch(&command,
                          &action_in_progress,
                          &active_seq,
                          &active_action,
                          &ack_cache,
                          &comm_safety,
                          &dispatch_ops);
}

static void service_motion(void)
{
    hand_command_service_motion(&action_in_progress,
                                &active_seq,
                                &active_action,
                                &ack_cache,
                                hand_servo_update,
                                hand_servo_is_busy,
                                record_ack);
}

static void expect_ack(size_t index, uint32_t seq, ack_status_t status)
{
    assert(index < ack_event_count);
    assert(ack_events[index].seq == seq);
    assert(ack_events[index].status == status);
}

static void expect_inactive(void)
{
    assert(action_in_progress == 0U);
    assert(active_seq == 0U);
    assert(active_action == HAND_ACTION_NONE);
}

static void test_stop_cancellation_has_no_completion_ack(void)
{
    reset_fixture();
    dispatch(101U, HAND_ACTION_GRAB);
    assert(action_in_progress == 1U);
    assert(active_seq == 101U);
    assert(active_action == HAND_ACTION_GRAB);
    assert(hand_ack_cache_find(&ack_cache, 101U)->status ==
           ACK_STATUS_IN_PROGRESS);

    dispatch(102U, HAND_ACTION_STOP);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(hand_ack_cache_find(&ack_cache, 101U)->status ==
           ACK_STATUS_PREEMPTED);
    assert(hand_ack_cache_find(&ack_cache, 102U)->status == ACK_STATUS_OK);
    assert(start_action_call_count == 2U);
    assert(stop_call_count == 1U);
    expect_inactive();
    assert(ack_event_count == 3U);
    expect_ack(0U, 101U, ACK_STATUS_IN_PROGRESS);
    expect_ack(1U, 101U, ACK_STATUS_PREEMPTED);
    expect_ack(2U, 102U, ACK_STATUS_OK);

    service_motion();
    assert(hand_ack_cache_find(&ack_cache, 101U)->status ==
           ACK_STATUS_PREEMPTED);
    assert(ack_event_count == 3U);
    assert(start_action_call_count == 2U);
    expect_inactive();
    assert(seq_conflict_count == 0U);
}

static void test_uart_cancellation_has_no_completion_ack(void)
{
    reset_fixture();
    dispatch(201U, HAND_ACTION_GRAB);
    hand_comm_safety_notify_uart_error_isr(&comm_safety);
    assert(hand_comm_safety_service(&comm_safety,
                                    &action_in_progress,
                                    &active_seq,
                                    &active_action,
                                    &ack_cache,
                                    counted_real_start_action) ==
           HAND_COMM_SAFETY_ACTION_CANCELLED);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(hand_ack_cache_find(&ack_cache, 201U)->status ==
           ACK_STATUS_PREEMPTED);
    assert(start_action_call_count == 2U);
    assert(stop_call_count == 1U);
    expect_inactive();
    assert(ack_event_count == 1U);
    expect_ack(0U, 201U, ACK_STATUS_IN_PROGRESS);

    service_motion();
    assert(hand_ack_cache_find(&ack_cache, 201U)->status ==
           ACK_STATUS_PREEMPTED);
    assert(ack_event_count == 1U);
    expect_inactive();

    dispatch(201U, HAND_ACTION_GRAB);
    assert(ack_event_count == 2U);
    expect_ack(1U, 201U, ACK_STATUS_PREEMPTED);
    assert(start_action_call_count == 2U);
    assert(seq_conflict_count == 0U);
}

static void test_grab_completes_once(void)
{
    reset_fixture();
    dispatch(301U, HAND_ACTION_GRAB);
    assert(action_in_progress == 1U);
    assert(active_seq == 301U);
    assert(active_action == HAND_ACTION_GRAB);
    assert(ack_event_count == 1U);
    expect_ack(0U, 301U, ACK_STATUS_IN_PROGRESS);
    assert(hand_ack_cache_find(&ack_cache, 301U)->status ==
           ACK_STATUS_IN_PROGRESS);

    for (uint32_t step = 0U; step < 12U && action_in_progress != 0U; step++)
    {
        hal_mock_advance_tick(20U);
        service_motion();
    }

    expect_inactive();
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(hand_ack_cache_find(&ack_cache, 301U)->status == ACK_STATUS_OK);
    assert(ack_event_count == 2U);
    expect_ack(1U, 301U, ACK_STATUS_OK);
    assert(start_action_call_count == 1U);

    service_motion();
    assert(ack_event_count == 2U);
    assert(hand_ack_cache_find(&ack_cache, 301U)->status == ACK_STATUS_OK);
    assert(seq_conflict_count == 0U);
}

static void test_completed_grab_replays_cached_ok(void)
{
    reset_fixture();
    dispatch(401U, HAND_ACTION_GRAB);
    for (uint32_t step = 0U; step < 12U && action_in_progress != 0U; step++)
    {
        hal_mock_advance_tick(20U);
        service_motion();
    }
    expect_inactive();
    assert(ack_event_count == 2U);
    assert(hand_ack_cache_find(&ack_cache, 401U)->status == ACK_STATUS_OK);

    dispatch(401U, HAND_ACTION_GRAB);
    assert(start_action_call_count == 1U);
    assert(ack_event_count == 3U);
    expect_ack(0U, 401U, ACK_STATUS_IN_PROGRESS);
    expect_ack(1U, 401U, ACK_STATUS_OK);
    expect_ack(2U, 401U, ACK_STATUS_OK);
    expect_inactive();
    service_motion();
    assert(ack_event_count == 3U);
    assert(seq_conflict_count == 0U);
}

/* FAULT is persistent in hand_servo.c, so this case must run last. */
static void test_i2c_error_is_terminal_and_survives_late_uart_error(void)
{
    reset_fixture();
    dispatch(501U, HAND_ACTION_GRAB);
    hal_mock_fail_next_i2c_write();
    hal_mock_advance_tick(20U);
    service_motion();

    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_ack_cache_find(&ack_cache, 501U)->status == ACK_STATUS_ERROR);
    expect_inactive();
    assert(ack_event_count == 2U);
    expect_ack(0U, 501U, ACK_STATUS_IN_PROGRESS);
    expect_ack(1U, 501U, ACK_STATUS_ERROR);
    assert(start_action_call_count == 1U);

    service_motion();
    assert(ack_event_count == 2U);
    assert(hand_ack_cache_find(&ack_cache, 501U)->status == ACK_STATUS_ERROR);

    /* Inject a stale activity snapshot to exercise the existing ERROR guard. */
    action_in_progress = 1U;
    active_seq = 501U;
    active_action = HAND_ACTION_GRAB;
    hand_comm_safety_notify_uart_error_isr(&comm_safety);
    assert(hand_comm_safety_service(&comm_safety,
                                    &action_in_progress,
                                    &active_seq,
                                    &active_action,
                                    &ack_cache,
                                    counted_real_start_action) ==
           HAND_COMM_SAFETY_ACTION_CANCELLED);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_ack_cache_find(&ack_cache, 501U)->status == ACK_STATUS_ERROR);
    expect_inactive();
    assert(ack_event_count == 2U);
    assert(start_action_call_count == 2U);
    assert(stop_call_count == 1U);
    assert(seq_conflict_count == 0U);
}

int main(void)
{
    test_stop_cancellation_has_no_completion_ack();
    test_uart_cancellation_has_no_completion_ack();
    test_grab_completes_once();
    test_completed_grab_replays_cached_ok();
    test_i2c_error_is_terminal_and_survives_late_uart_error();

    puts("hand_command_lifecycle: all tests passed");
    return 0;
}
