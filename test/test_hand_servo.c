#include <assert.h>
#include <stdint.h>

#include "hand_servo.h"

void hal_mock_reset(void);
void hal_mock_advance_tick(uint32_t milliseconds);
uint32_t hal_mock_get_i2c_write_count(void);
void hal_mock_fail_next_i2c_write(void);

int main(void)
{
    I2C_HandleTypeDef mock_i2c = {0U};

    hal_mock_reset();

    /* Exercise initialization and actions from the actual production sources. */
    assert(hand_servo_init(&mock_i2c) == HAL_OK);

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

    return 0;
}
