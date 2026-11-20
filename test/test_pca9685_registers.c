#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hal_mock_transactions.h"
#include "pca9685.h"

void hal_mock_reset(void);
uint32_t hal_mock_get_tick(void);
uint32_t hal_mock_get_i2c_write_count(void);
void hal_mock_fail_next_i2c_write(void);
void hal_mock_fail_i2c_write_on_nth_from_now(uint32_t nth_write);

static I2C_HandleTypeDef mock_i2c;

static const hal_mock_i2c_transaction_t *transaction_at(size_t index)
{
    assert(index < hal_mock_i2c_transaction_count());
    const hal_mock_i2c_transaction_t *entry =
        hal_mock_i2c_transaction_at(index);
    assert(entry != NULL);
    assert(entry->handle == &mock_i2c);
    assert(entry->device_address == 0x80U);
    assert(entry->register_address_size == I2C_MEMADD_SIZE_8BIT);
    assert(entry->timeout == 100U);
    return entry;
}

static void expect_write(size_t index,
                         uint16_t register_address,
                         const uint8_t *data,
                         uint16_t length,
                         HAL_StatusTypeDef result)
{
    const hal_mock_i2c_transaction_t *entry = transaction_at(index);
    assert(entry->register_address == register_address);
    assert(entry->data_length == length);
    assert(length <= HAL_MOCK_I2C_PAYLOAD_CAPACITY);
    assert(memcmp(entry->data, data, length) == 0);
    assert(entry->result == result);
}

static void expect_no_recording_overflow(void)
{
    assert(hal_mock_i2c_transaction_overflow_count() == 0U);
    assert(hal_mock_i2c_payload_overflow_count() == 0U);
}

static void prepare_initialized(void)
{
    hal_mock_reset();
    assert(pca9685_init(&mock_i2c) == HAL_OK);
    assert(hal_mock_i2c_transaction_count() == 4U);
    expect_no_recording_overflow();
    hal_mock_i2c_transactions_reset();
}

/* NXP PCA9685 Rev. 4, Table 5 (MODE1), Section 7.3.5 (PRE_SCALE). */
static void test_initialization_records_current_sequence(void)
{
    static const uint8_t sleep[] = {0x10U};
    static const uint8_t prescale[] = {121U};
    static const uint8_t wake_ai[] = {0x20U};
    static const uint8_t restart_ai_allcall[] = {0xA1U};

    hal_mock_reset();
    assert(pca9685_init(NULL) == HAL_ERROR);
    assert(hal_mock_i2c_transaction_count() == 0U);

    assert(pca9685_init(&mock_i2c) == HAL_OK);
    assert(hal_mock_i2c_transaction_count() == 4U);
    assert(hal_mock_get_i2c_write_count() == 4U);
    expect_write(0U, 0x00U, sleep, 1U, HAL_OK);
    expect_write(1U, 0xFEU, prescale, 1U, HAL_OK);
    expect_write(2U, 0x00U, wake_ai, 1U, HAL_OK);
    expect_write(3U, 0x00U, restart_ai_allcall, 1U, HAL_OK);
    assert((restart_ai_allcall[0] & 0x80U) != 0U);
    assert((restart_ai_allcall[0] & 0x20U) != 0U);
    assert((restart_ai_allcall[0] & 0x01U) != 0U);
    assert(hal_mock_get_tick() == 10U);
    expect_no_recording_overflow();
}

/* NXP Table 4 and Table 7: 06h + 4 * channel, then four PWM bytes. */
static void test_pwm_registers_and_byte_order(void)
{
    static const uint8_t channel_zero[] = {0x23U, 0x01U, 0xBCU, 0x0AU};
    static const uint8_t channel_four[] = {0x00U, 0x00U, 0x33U, 0x01U};
    static const uint8_t channel_fifteen[] = {0xFFU, 0x0FU, 0x00U, 0x00U};

    prepare_initialized();
    assert(pca9685_set_pwm(0U, 0x123U, 0xABCU) == HAL_OK);
    assert(pca9685_set_pwm(4U, 0U, 0x133U) == HAL_OK);
    assert(pca9685_set_pwm(15U, 4095U, 0U) == HAL_OK);
    assert(hal_mock_i2c_transaction_count() == 3U);
    expect_write(0U, 0x06U, channel_zero, 4U, HAL_OK);
    expect_write(1U, 0x16U, channel_four, 4U, HAL_OK);
    expect_write(2U, 0x42U, channel_fifteen, 4U, HAL_OK);
    expect_no_recording_overflow();
}

/* NXP Section 7.3.3: 12-bit counts; pulse mapping is this project's policy. */
static void test_angle_to_actual_pwm_bytes(void)
{
    static const uint8_t angle_zero[] = {0x00U, 0x00U, 0x66U, 0x00U};
    static const uint8_t angle_ninety[] = {0x00U, 0x00U, 0x33U, 0x01U};
    static const uint8_t angle_one_eighty[] = {0x00U, 0x00U, 0x00U, 0x02U};

    prepare_initialized();
    assert(pca9685_set_servo_angle(0U, 0U) == HAL_OK);
    assert(pca9685_set_servo_angle(1U, 90U) == HAL_OK);
    assert(pca9685_set_servo_angle(4U, 180U) == HAL_OK);
    assert(hal_mock_i2c_transaction_count() == 3U);
    expect_write(0U, 0x06U, angle_zero, 4U, HAL_OK);     /* 500 us -> 102 */
    expect_write(1U, 0x0AU, angle_ninety, 4U, HAL_OK);   /* 1500 us -> 307 */
    expect_write(2U, 0x16U, angle_one_eighty, 4U, HAL_OK); /* 2500 us -> 512 */
    expect_no_recording_overflow();
}

/* NXP Table 7 limits ON/OFF counts to 0..4095. */
static void test_channel_and_pwm_count_boundaries(void)
{
    static const uint8_t max_valid[] = {0x00U, 0x00U, 0xFFU, 0x0FU};
    static const uint8_t equal_valid[] = {0xFFU, 0x0FU, 0xFFU, 0x0FU};

    prepare_initialized();
    assert(pca9685_set_pwm(15U, 0U, 4095U) == HAL_OK);
    expect_write(0U, 0x42U, max_valid, 4U, HAL_OK);

    /* ON == OFF remains accepted by this API in this change. */
    assert(pca9685_set_pwm(0U, 4095U, 4095U) == HAL_OK);
    expect_write(1U, 0x06U, equal_valid, 4U, HAL_OK);
    assert(hal_mock_i2c_transaction_count() == 2U);

    uint32_t writes_before_invalid = hal_mock_get_i2c_write_count();

    assert(pca9685_set_pwm(16U, 0U, 1U) == HAL_ERROR);
    assert(hal_mock_i2c_transaction_count() == 2U);
    assert(hal_mock_get_i2c_write_count() == writes_before_invalid);

    assert(pca9685_set_pwm(0U, 4096U, 1U) == HAL_ERROR);
    assert(hal_mock_i2c_transaction_count() == 2U);
    assert(hal_mock_get_i2c_write_count() == writes_before_invalid);

    assert(pca9685_set_pwm(0U, 0U, 5000U) == HAL_ERROR);
    assert(hal_mock_i2c_transaction_count() == 2U);
    assert(hal_mock_get_i2c_write_count() == writes_before_invalid);

    assert(pca9685_set_pwm(0U, 4096U, 5000U) == HAL_ERROR);
    assert(hal_mock_i2c_transaction_count() == 2U);
    assert(hal_mock_get_i2c_write_count() == writes_before_invalid);
    expect_no_recording_overflow();
}

static void test_failed_i2c_write_is_recorded_without_changing_injection(void)
{
    static const uint8_t payload[] = {0x00U, 0x00U, 0x66U, 0x00U};

    prepare_initialized();
    hal_mock_fail_next_i2c_write();
    assert(pca9685_set_servo_angle(0U, 0U) == HAL_ERROR);
    assert(pca9685_set_servo_angle(0U, 0U) == HAL_OK);
    assert(hal_mock_i2c_transaction_count() == 2U);
    expect_write(0U, 0x06U, payload, 4U, HAL_ERROR);
    expect_write(1U, 0x06U, payload, 4U, HAL_OK);
    expect_no_recording_overflow();

    hal_mock_reset();
    hal_mock_fail_i2c_write_on_nth_from_now(2U);
    assert(pca9685_init(&mock_i2c) == HAL_ERROR);
    assert(hal_mock_i2c_transaction_count() == 2U);
    assert(transaction_at(0U)->result == HAL_OK);
    assert(transaction_at(1U)->result == HAL_ERROR);
    expect_no_recording_overflow();
}

static void test_transaction_log_reports_capacity_and_reset(void)
{
    prepare_initialized();
    for (size_t i = 0U; i < HAL_MOCK_I2C_TRANSACTION_CAPACITY; i++)
    {
        assert(pca9685_set_pwm(0U, 0U, 1U) == HAL_OK);
    }
    assert(hal_mock_i2c_transaction_count() ==
           HAL_MOCK_I2C_TRANSACTION_CAPACITY);
    assert(hal_mock_i2c_transaction_overflow_count() == 0U);

    assert(pca9685_set_pwm(0U, 0U, 1U) == HAL_OK);
    assert(hal_mock_i2c_transaction_count() ==
           HAL_MOCK_I2C_TRANSACTION_CAPACITY);
    assert(hal_mock_i2c_transaction_overflow_count() == 1U);
    assert(hal_mock_get_i2c_write_count() ==
           4U + HAL_MOCK_I2C_TRANSACTION_CAPACITY + 1U);

    hal_mock_i2c_transactions_reset();
    assert(hal_mock_i2c_transaction_count() == 0U);
    assert(hal_mock_i2c_transaction_overflow_count() == 0U);
    assert(hal_mock_i2c_payload_overflow_count() == 0U);
}

int main(void)
{
    test_initialization_records_current_sequence();
    test_pwm_registers_and_byte_order();
    test_angle_to_actual_pwm_bytes();
    test_channel_and_pwm_count_boundaries();
    test_failed_i2c_write_is_recorded_without_changing_injection();
    test_transaction_log_reports_capacity_and_reset();

    puts("pca9685_registers: all assertions passed");
    return 0;
}
