#include <assert.h>
#include <stdint.h>

#include "hand_servo.h"

void hal_mock_reset(void);
void hal_mock_set_tick(uint32_t tick);
void hal_mock_advance_tick(uint32_t milliseconds);
void hal_mock_set_i2c_write_duration(uint32_t milliseconds);
uint32_t hal_mock_get_i2c_write_count(void);
uint32_t hal_mock_get_tick(void);
void hal_mock_fail_next_i2c_write(void);

static I2C_HandleTypeDef mock_i2c;

static void prepare_servo_fixture(void)
{
    hal_mock_reset();
    assert(hand_servo_init(&mock_i2c) == HAL_OK);
}

static void test_normal_actions_and_stop(void)
{
    prepare_servo_fixture();

    /* Start OPEN and let one asynchronous update reach the PCA9685 mock. */
    assert(hand_servo_start_action(HAND_ACTION_OPEN) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_OPENING);
    assert(hand_servo_is_busy() != 0U);
    hal_mock_advance_tick(20U);
    assert(hand_servo_update() == HAL_OK);

    /* STOP cancels the trajectory and IDLE updates make no further writes. */
    uint32_t writes_before_stop = hal_mock_get_i2c_write_count();
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(hand_servo_is_busy() == 0U);
    assert(hal_mock_get_i2c_write_count() == writes_before_stop);
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before_stop);

    /* Ordinary actions remain admissible outside FAULT. */
    assert(hand_servo_start_action(HAND_ACTION_GRAB) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_GRABBING);
    assert(hand_servo_is_busy() != 0U);
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
}

static void test_interval_19ms_and_20ms_boundary(void)
{
    prepare_servo_fixture();
    assert(hand_servo_start_action(HAND_ACTION_OPEN) == HAL_OK);

    uint32_t writes_before = hal_mock_get_i2c_write_count();
    hal_mock_advance_tick(19U);
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before);

    hal_mock_advance_tick(1U);
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before + HAND_SERVO_NUM);
    assert(hand_servo_get_state() == HAND_MOTION_OPENING);
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
}

static void test_tick_wraparound(void)
{
    prepare_servo_fixture();
    hal_mock_set_tick(UINT32_MAX - 10U);
    assert(hand_servo_start_action(HAND_ACTION_OPEN) == HAL_OK);

    uint32_t writes_before = hal_mock_get_i2c_write_count();
    hal_mock_advance_tick(19U); /* Wraps to 8; elapsed time is still 19 ms. */
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before);

    hal_mock_advance_tick(1U); /* Wraps to 9; elapsed time is exactly 20 ms. */
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before + HAND_SERVO_NUM);
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
}

static void test_long_delay_advances_only_one_round(void)
{
    prepare_servo_fixture();
    assert(hand_servo_start_action(HAND_ACTION_OPEN) == HAL_OK);

    /* The reset fixture starts at 90 degrees; OPEN targets 40 degrees. */
    uint32_t writes_before = hal_mock_get_i2c_write_count();
    hal_mock_advance_tick(1000U);
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before + HAND_SERVO_NUM);
    assert(hand_servo_get_state() == HAND_MOTION_OPENING);

    /* There is ample trajectory remaining; another call at the same tick waits. */
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before + HAND_SERVO_NUM);
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
}

static void test_slow_i2c_requires_20ms_after_batch_completion(void)
{
    prepare_servo_fixture();
    assert(hand_servo_start_action(HAND_ACTION_OPEN) == HAL_OK);

    /* One update changes all five channels; each mocked write consumes 5 ms. */
    hal_mock_advance_tick(20U);
    hal_mock_set_i2c_write_duration(5U);
    uint32_t writes_before = hal_mock_get_i2c_write_count();
    uint32_t tick_before = hal_mock_get_tick();

    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_before + HAND_SERVO_NUM);
    assert((uint32_t)(hal_mock_get_tick() - tick_before) == 25U);
    assert(hand_servo_get_state() == HAND_MOTION_OPENING);

    /*
     * Proposed cadence: the next round must wait 20 ms from the last write's
     * completion. The current production code is expected to fail this check.
     */
    uint32_t writes_after_batch = hal_mock_get_i2c_write_count();
    uint32_t completion_tick = hal_mock_get_tick();
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_after_batch);
    assert(hal_mock_get_tick() == completion_tick);

    hal_mock_advance_tick(19U);
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_after_batch);

    hal_mock_advance_tick(1U);
    assert(hand_servo_update() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() == writes_after_batch + HAND_SERVO_NUM);
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
}

static void test_i2c_failure_and_fault_stop(void)
{
    prepare_servo_fixture();

    /* An actual PCA write failure in update must latch FAULT. */
    assert(hand_servo_start_action(HAND_ACTION_RELEASE) == HAL_OK);
    hal_mock_advance_tick(20U);
    uint32_t writes_before_fault = hal_mock_get_i2c_write_count();
    hal_mock_fail_next_i2c_write();
    assert(hand_servo_update() == HAL_ERROR);
    assert(hal_mock_get_i2c_write_count() == writes_before_fault + 1U);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_servo_is_busy() == 0U);

    /* STOP in FAULT cancels software progression but preserves FAULT. */
    uint32_t writes_in_fault = hal_mock_get_i2c_write_count();
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hal_mock_get_i2c_write_count() == writes_in_fault);
    assert(hand_servo_update() == HAL_ERROR);
    assert(hal_mock_get_i2c_write_count() == writes_in_fault);

    /* All ordinary actions are rejected without changing the state or bus. */
    assert(hand_servo_start_action(HAND_ACTION_OPEN) == HAL_ERROR);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_servo_start_action(HAND_ACTION_GRAB) == HAL_ERROR);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_servo_start_action(HAND_ACTION_RELEASE) == HAL_ERROR);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hal_mock_get_i2c_write_count() == writes_in_fault);
}

int main(void)
{
    test_normal_actions_and_stop();
    test_interval_19ms_and_20ms_boundary();
    test_tick_wraparound();
    test_long_delay_advances_only_one_round();
    test_slow_i2c_requires_20ms_after_batch_completion();
    test_i2c_failure_and_fault_stop();

    return 0;
}
