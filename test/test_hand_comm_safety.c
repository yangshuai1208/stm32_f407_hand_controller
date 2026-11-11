#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "hand_comm_safety.h"

void hal_mock_reset(void);
void hal_mock_advance_tick(uint32_t milliseconds);
void hal_mock_fail_next_i2c_write(void);
uint32_t hal_mock_get_i2c_write_count(void);
uint32_t hal_mock_get_primask(void);
void hal_mock_set_primask(uint32_t primask);

static I2C_HandleTypeDef mock_i2c;
static hand_comm_safety_t safety;
static hand_ack_cache_t cache;
static uint8_t action_in_progress;
static uint32_t active_seq;
static hand_action_t active_action;
static uint32_t stop_call_count;

static HAL_StatusTypeDef counted_real_start_action(hand_action_t action)
{
    if (action == HAND_ACTION_STOP)
    {
        stop_call_count++;
    }
    return hand_servo_start_action(action);
}

static HAL_StatusTypeDef failing_start_action(hand_action_t action)
{
    assert(action == HAND_ACTION_STOP);
    stop_call_count++;
    return HAL_ERROR;
}

static void reset_fixture(void)
{
    hal_mock_reset();
    hand_comm_safety_init(&safety);
    hand_ack_cache_init(&cache);
    action_in_progress = 0U;
    active_seq = 0U;
    active_action = HAND_ACTION_NONE;
    stop_call_count = 0U;
    assert(hand_servo_init(&mock_i2c) == HAL_OK);
}

static void start_grab(uint32_t seq)
{
    assert(hand_servo_start_action(HAND_ACTION_GRAB) == HAL_OK);
    hand_comm_safety_on_action_started(&safety);
    action_in_progress = 1U;
    active_seq = seq;
    active_action = HAND_ACTION_GRAB;
    hand_ack_cache_set(&cache, seq, HAND_ACTION_GRAB,
                       ACK_STATUS_IN_PROGRESS);
}

static hand_comm_safety_result_t service_safety(
    hand_comm_start_action_fn start_action)
{
    return hand_comm_safety_service(&safety,
                                    &action_in_progress,
                                    &active_seq,
                                    &active_action,
                                    &cache,
                                    start_action);
}

static void test_first_uart_error_cancels_and_caches_preempted(void)
{
    reset_fixture();
    start_grab(101U);
    uint32_t writes_before = hal_mock_get_i2c_write_count();

    hand_comm_safety_notify_uart_error_isr(&safety);
    assert(service_safety(counted_real_start_action) ==
           HAND_COMM_SAFETY_ACTION_CANCELLED);

    assert(stop_call_count == 1U);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(hal_mock_get_i2c_write_count() == writes_before);
    assert(action_in_progress == 0U);
    assert(active_seq == 0U);
    assert(active_action == HAND_ACTION_NONE);
    assert(hand_ack_cache_find(&cache, 101U)->status ==
           ACK_STATUS_PREEMPTED);
}

static void test_fast_recovery_does_not_resume_cancelled_grab(void)
{
    reset_fixture();
    start_grab(102U);
    hand_comm_safety_notify_uart_error_isr(&safety);
    assert(service_safety(counted_real_start_action) ==
           HAND_COMM_SAFETY_ACTION_CANCELLED);

    /* Recovery becoming healthy does not recreate the cancelled trajectory. */
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(action_in_progress == 0U);
    assert(hand_ack_cache_find(&cache, 102U)->status ==
           ACK_STATUS_PREEMPTED);
}

static void test_repeated_errors_do_not_stop_same_action_twice(void)
{
    reset_fixture();
    start_grab(103U);
    hand_comm_safety_notify_uart_error_isr(&safety);
    assert(service_safety(counted_real_start_action) ==
           HAND_COMM_SAFETY_ACTION_CANCELLED);

    hand_comm_safety_notify_uart_error_isr(&safety);
    assert(service_safety(counted_real_start_action) ==
           HAND_COMM_SAFETY_NO_ACTIVE_ACTION);
    assert(stop_call_count == 1U);
}

static void test_servo_fault_and_existing_error_terminal_are_preserved(void)
{
    reset_fixture();
    start_grab(104U);
    hal_mock_advance_tick(20U);
    hal_mock_fail_next_i2c_write();
    assert(hand_servo_update() == HAL_ERROR);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    hand_ack_cache_set(&cache, 104U, HAND_ACTION_GRAB,
                       ACK_STATUS_ERROR);

    hand_comm_safety_notify_uart_error_isr(&safety);
    assert(service_safety(counted_real_start_action) ==
           HAND_COMM_SAFETY_ACTION_CANCELLED);

    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_ack_cache_find(&cache, 104U)->status == ACK_STATUS_ERROR);
    assert(action_in_progress == 0U);
    assert(active_seq == 0U);
    assert(active_action == HAND_ACTION_NONE);
}

static void test_stop_failure_does_not_cache_preempted_or_clear_activity(void)
{
    reset_fixture();
    start_grab(105U);
    hand_comm_safety_notify_uart_error_isr(&safety);
    assert(service_safety(failing_start_action) ==
           HAND_COMM_SAFETY_STOP_FAILED);

    assert(stop_call_count == 1U);
    assert(action_in_progress == 1U);
    assert(active_seq == 105U);
    assert(active_action == HAND_ACTION_GRAB);
    assert(hand_ack_cache_find(&cache, 105U)->status ==
           ACK_STATUS_IN_PROGRESS);

    hand_comm_safety_notify_uart_error_isr(&safety);
    assert(service_safety(failing_start_action) ==
           HAND_COMM_SAFETY_ALREADY_ATTEMPTED);
    assert(stop_call_count == 1U);
    assert(hand_ack_cache_find(&cache, 105U)->status !=
           ACK_STATUS_PREEMPTED);
}

static void test_error_without_active_action_does_not_call_stop(void)
{
    reset_fixture();
    hand_comm_safety_notify_uart_error_isr(&safety);

    assert(service_safety(counted_real_start_action) ==
           HAND_COMM_SAFETY_NO_ACTIVE_ACTION);
    assert(stop_call_count == 0U);
    assert(action_in_progress == 0U);
}

static void test_event_take_preserves_existing_interrupt_mask(void)
{
    reset_fixture();
    hand_comm_safety_notify_uart_error_isr(&safety);
    hal_mock_set_primask(1U);

    assert(service_safety(counted_real_start_action) ==
           HAND_COMM_SAFETY_NO_ACTIVE_ACTION);
    assert(hal_mock_get_primask() == 1U);

    hal_mock_set_primask(0U);
}

int main(void)
{
    test_first_uart_error_cancels_and_caches_preempted();
    test_fast_recovery_does_not_resume_cancelled_grab();
    test_repeated_errors_do_not_stop_same_action_twice();
    test_stop_failure_does_not_cache_preempted_or_clear_activity();
    test_error_without_active_action_does_not_call_stop();
    test_event_take_preserves_existing_interrupt_mask();
    test_servo_fault_and_existing_error_terminal_are_preserved();

    puts("hand_comm_safety: all tests passed");
    return 0;
}
