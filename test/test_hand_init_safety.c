#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hand_command_dispatch.h"
#include "hand_servo.h"

void hal_mock_reset(void);
void hal_mock_advance_tick(uint32_t milliseconds);
void hal_mock_fail_i2c_write_on_nth_from_now(uint32_t nth_write);
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
static ack_event_t ack_events[16];
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

static void prepare(void)
{
    hal_mock_reset();
    hand_ack_cache_init(&ack_cache);
    hand_comm_safety_init(&comm_safety);
    action_in_progress = 0U;
    active_seq = 0U;
    active_action = HAND_ACTION_NONE;
    ack_event_count = 0U;
    start_call_count = 0U;
    conflict_count = 0U;
}

static void dispatch_line(const char *line)
{
    hand_command_t command;
    assert(hand_protocol_parse_frame(line, &command));
    hand_command_dispatch(&command, &action_in_progress, &active_seq,
                          &active_action, &ack_cache, &comm_safety,
                          &dispatch_ops);
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

static void test_normal_initialization(void)
{
    prepare();
    assert(hand_servo_init(&mock_i2c) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);

    dispatch_line("SEQ:1 CMD:HAND_OPEN");
    assert(start_call_count == 1U);
    assert(action_in_progress == 1U);
    assert(active_seq == 1U);
    assert(active_action == HAND_ACTION_OPEN);
    assert(hand_servo_get_state() == HAND_MOTION_OPENING);
    assert(hand_ack_cache_find(&ack_cache, 1U)->status ==
           ACK_STATUS_IN_PROGRESS);
    expect_ack(0U, 1U, ACK_STATUS_IN_PROGRESS);

    uint32_t writes_before_update = hal_mock_get_i2c_write_count();
    hal_mock_advance_tick(20U);
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() > writes_before_update);

    dispatch_line("SEQ:2 CMD:HAND_STOP");
    assert(start_call_count == 2U);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    expect_inactive();
    assert(hand_ack_cache_find(&ack_cache, 1U)->status ==
           ACK_STATUS_PREEMPTED);
    assert(hand_ack_cache_find(&ack_cache, 2U)->status == ACK_STATUS_OK);
    assert(ack_event_count == 3U);
    expect_ack(1U, 1U, ACK_STATUS_PREEMPTED);
    expect_ack(2U, 2U, ACK_STATUS_OK);
}

static void test_failed_initialization(uint32_t failing_write,
                                       uint32_t expected_writes)
{
    prepare();
    hal_mock_fail_i2c_write_on_nth_from_now(failing_write);
    assert(hand_servo_init(&mock_i2c) == HAL_ERROR);
    assert(hal_mock_get_i2c_write_count() == expected_writes);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_servo_is_busy() == 0U);

    uint32_t writes_after_init = hal_mock_get_i2c_write_count();
    dispatch_line("SEQ:11 CMD:HAND_OPEN");
    dispatch_line("SEQ:12 CMD:HAND_GRAB");
    dispatch_line("SEQ:13 CMD:HAND_RELEASE");
    assert(start_call_count == 3U);
    expect_inactive();
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(ack_event_count == 3U);
    for (uint32_t seq = 11U; seq <= 13U; seq++)
    {
        expect_ack(seq - 11U, seq, ACK_STATUS_ERROR);
        assert(hand_ack_cache_find(&ack_cache, seq)->status ==
               ACK_STATUS_ERROR);
    }
    assert(hal_mock_get_i2c_write_count() == writes_after_init);

    dispatch_line("SEQ:11 CMD:HAND_OPEN");
    assert(start_call_count == 3U);
    expect_ack(3U, 11U, ACK_STATUS_ERROR);
    dispatch_line("SEQ:11 CMD:HAND_GRAB");
    assert(start_call_count == 3U);
    assert(conflict_count == 1U);
    assert(ack_event_count == 4U);

    dispatch_line("SEQ:14 CMD:HAND_STOP");
    assert(start_call_count == 4U);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    expect_inactive();
    assert(hand_ack_cache_find(&ack_cache, 14U)->status == ACK_STATUS_OK);
    expect_ack(4U, 14U, ACK_STATUS_OK);
    assert(hal_mock_get_i2c_write_count() == writes_after_init);

    dispatch_line("SEQ:15 CMD:HAND_GRAB");
    assert(start_call_count == 5U);
    expect_ack(5U, 15U, ACK_STATUS_ERROR);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    expect_inactive();
    assert(hal_mock_get_i2c_write_count() == writes_after_init);

    hand_comm_safety_notify_uart_error_isr(&comm_safety);
    assert(hand_comm_safety_service(&comm_safety, &action_in_progress,
                                    &active_seq, &active_action, &ack_cache,
                                    counted_real_start) ==
           HAND_COMM_SAFETY_NO_ACTIVE_ACTION);
    assert(start_call_count == 5U);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(ack_event_count == 6U);
    assert(hal_mock_get_i2c_write_count() == writes_after_init);

    assert(hand_servo_update() == HAL_ERROR);
    assert(hal_mock_get_i2c_write_count() == writes_after_init);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (strcmp(argv[1], "normal") == 0)
    {
        test_normal_initialization();
    }
    else if (strcmp(argv[1], "pca_failure") == 0)
    {
        test_failed_initialization(1U, 1U);
    }
    else if (strcmp(argv[1], "servo_failure") == 0)
    {
        /* Four PCA9685 configuration writes, then three channel writes. */
        test_failed_initialization(7U, 7U);
    }
    else
    {
        assert(0 && "unknown scenario");
    }

    printf("hand_init_safety %s: all assertions passed\n", argv[1]);
    return 0;
}
