#ifndef HAL_MOCK_TRANSACTIONS_H
#define HAL_MOCK_TRANSACTIONS_H

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define HAL_MOCK_I2C_TRANSACTION_CAPACITY 128U
#define HAL_MOCK_I2C_PAYLOAD_CAPACITY 32U

/* The payload is the byte array passed to HAL, including failed attempts. */
typedef struct
{
    I2C_HandleTypeDef *handle;
    uint16_t device_address;
    uint16_t register_address;
    uint16_t register_address_size;
    uint16_t data_length;
    uint8_t data[HAL_MOCK_I2C_PAYLOAD_CAPACITY];
    uint32_t timeout;
    HAL_StatusTypeDef result;
} hal_mock_i2c_transaction_t;

void hal_mock_i2c_transactions_reset(void);
size_t hal_mock_i2c_transaction_count(void);
const hal_mock_i2c_transaction_t *hal_mock_i2c_transaction_at(size_t index);
size_t hal_mock_i2c_transaction_overflow_count(void);
size_t hal_mock_i2c_payload_overflow_count(void);

#endif
