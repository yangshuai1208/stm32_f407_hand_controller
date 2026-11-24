#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hal_mock_transactions.h"
#include "hand_servo.h"

/* Compile the real module in this test translation unit to inspect its
 * private angle and target arrays without adding production accessors. */
#include "../Core/Src/hand_servo.c"

void hal_mock_reset(void);
void hal_mock_advance_tick(uint32_t milliseconds);
void hal_mock_fail_i2c_write_on_nth_from_now(uint32_t nth_write);
uint32_t hal_mock_get_i2c_write_count(void);
uint32_t hal_mock_get_tick(void);

static I2C_HandleTypeDef mock_i2c;

static void enter_fault_on_third_grab_write(void)
{
    hal_mock_reset();
    assert(hand_servo_init(&mock_i2c) == HAL_OK);
    assert(hand_servo_start_action(HAND_ACTION_GRAB) == HAL_OK);
    hal_mock_advance_tick(20U);

    uint32_t writes_before = hal_mock_get_i2c_write_count();
    size_t transactions_before = hal_mock_i2c_transaction_count();
    hal_mock_fail_i2c_write_on_nth_from_now(3U);
    assert(hand_servo_update() == HAL_ERROR);
    assert(hal_mock_get_i2c_write_count() == writes_before + 3U);
    assert(hal_mock_i2c_transaction_count() == transactions_before + 3U);
    assert(hal_mock_i2c_transaction_at(transactions_before)->result == HAL_OK);
    assert(hal_mock_i2c_transaction_at(transactions_before + 1U)->result == HAL_OK);
    assert(hal_mock_i2c_transaction_at(transactions_before + 2U)->result == HAL_ERROR);
    assert(hal_mock_i2c_transaction_at(transactions_before + 2U)->register_address == 0x0EU);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hand_servo_is_busy() == 0U);
    assert(current_angles[0] == 95U && current_angles[1] == 95U);
    assert(current_angles[2] == 90U && current_angles[3] == 90U &&
           current_angles[4] == 90U);
    for (uint8_t i = 0U; i < HAND_SERVO_NUM; i++)
    {
        assert(target_angles[i] == 120U);
    }

    /* The one-shot I2C fault has expired; later writes would now succeed. */
    hal_mock_i2c_transactions_reset();
    assert(hal_mock_i2c_transaction_count() == 0U);
}

static void test_fault_guard(const char *mode)
{
    enter_fault_on_third_grab_write();
    uint32_t writes_before = hal_mock_get_i2c_write_count();
    uint32_t tick_before = hal_mock_get_tick();
    uint16_t current_before[HAND_SERVO_NUM];
    uint16_t target_before[HAND_SERVO_NUM];
    memcpy(current_before, current_angles, sizeof(current_before));
    memcpy(target_before, target_angles, sizeof(target_before));
    HAL_StatusTypeDef result;

    /* Current angles are 95,95,90,90,90. Each target differs. */
    if (strcmp(mode, "apply_action") == 0)
    {
        result = hand_servo_apply_action(HAND_ACTION_OPEN);
    }
    else if (strcmp(mode, "set_finger_angle") == 0)
    {
        result = hand_servo_set_finger_angle(HAND_FINGER_THUMB, 40U);
    }
    else
    {
        assert(strcmp(mode, "set_all_stop") == 0);
        result = hand_servo_set_all_stop();
    }

    printf("mode=%s ret=%d new_i2c=%u new_transactions=%zu elapsed_mock_ms=%u state=%d\n",
           mode, (int)result,
           (unsigned)(hal_mock_get_i2c_write_count() - writes_before),
           hal_mock_i2c_transaction_count(),
           (unsigned)(hal_mock_get_tick() - tick_before),
           (int)hand_servo_get_state());
    fflush(stdout);

    assert(result == HAL_ERROR);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hal_mock_i2c_transaction_count() == 0U);
    assert(hal_mock_i2c_transaction_overflow_count() == 0U);
    assert(hal_mock_get_i2c_write_count() == writes_before);
    assert(hal_mock_get_tick() == tick_before);
    assert(memcmp(current_angles, current_before, sizeof(current_before)) == 0);
    assert(memcmp(target_angles, target_before, sizeof(target_before)) == 0);
    assert(hand_servo_update() == HAL_ERROR);
    assert(hal_mock_get_i2c_write_count() == writes_before);

    /* Protocol STOP is hand_servo_start_action(STOP), not set_all_stop(). */
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_FAULT);
    assert(hal_mock_get_i2c_write_count() == writes_before);
    assert(hal_mock_get_tick() == tick_before);
}

static void test_healthy_sync_interfaces(void)
{
    hal_mock_reset();
    assert(hand_servo_init(&mock_i2c) == HAL_OK);
    uint32_t writes_before = hal_mock_get_i2c_write_count();
    uint32_t tick_before = hal_mock_get_tick();

    assert(hand_servo_apply_action(HAND_ACTION_OPEN) == HAL_OK);
    assert(hal_mock_get_i2c_write_count() > writes_before);
    assert(hal_mock_get_tick() > tick_before);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);

    writes_before = hal_mock_get_i2c_write_count();
    assert(hand_servo_set_finger_angle(HAND_FINGER_THUMB, 70U) == HAL_OK);
    assert(hal_mock_get_i2c_write_count() > writes_before);

    writes_before = hal_mock_get_i2c_write_count();
    assert(hand_servo_set_all_stop() == HAL_OK);
    assert(hal_mock_get_i2c_write_count() > writes_before);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
    assert(hand_servo_start_action(HAND_ACTION_STOP) == HAL_OK);
    assert(hand_servo_get_state() == HAND_MOTION_IDLE);
}

int main(int argc, char *argv[])
{
    assert(argc == 2);
    if (strcmp(argv[1], "healthy") == 0)
    {
        test_healthy_sync_interfaces();
    }
    else
    {
        test_fault_guard(argv[1]);
    }
    puts("test_hand_fault_guard: PASS");
    return 0;
}
