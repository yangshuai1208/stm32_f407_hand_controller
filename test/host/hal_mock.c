#include "stm32f4xx_hal.h"
#include "hal_mock_transactions.h"

#include <stdio.h>
#include <string.h>

static uint32_t fake_tick;
static uint32_t fake_primask;
static uint32_t i2c_write_count;
static uint32_t i2c_write_duration_ms;
static uint8_t fail_next_i2c_write;
static uint32_t fail_i2c_write_countdown;
static HAL_StatusTypeDef uart_abort_result;
static HAL_StatusTypeDef uart_start_result;
static uint32_t uart_abort_count;
static uint32_t uart_start_count;
static uint32_t uart_clear_overrun_count;
static hal_mock_i2c_transaction_t i2c_transactions[HAL_MOCK_I2C_TRANSACTION_CAPACITY];
static size_t i2c_transaction_count;
static size_t i2c_transaction_overflow_count;
static size_t i2c_payload_overflow_count;

void hal_mock_i2c_transactions_reset(void)
{
    i2c_transaction_count = 0U;
    i2c_transaction_overflow_count = 0U;
    i2c_payload_overflow_count = 0U;
    memset(i2c_transactions, 0, sizeof(i2c_transactions));
}

size_t hal_mock_i2c_transaction_count(void)
{
    return i2c_transaction_count;
}

const hal_mock_i2c_transaction_t *hal_mock_i2c_transaction_at(size_t index)
{
    if (index >= i2c_transaction_count)
    {
        return NULL;
    }
    return &i2c_transactions[index];
}

size_t hal_mock_i2c_transaction_overflow_count(void)
{
    return i2c_transaction_overflow_count;
}

size_t hal_mock_i2c_payload_overflow_count(void)
{
    return i2c_payload_overflow_count;
}

static void hal_mock_record_i2c_write(I2C_HandleTypeDef *hi2c,
                                      uint16_t device_address,
                                      uint16_t register_address,
                                      uint16_t register_address_size,
                                      const uint8_t *data,
                                      uint16_t data_length,
                                      uint32_t timeout,
                                      HAL_StatusTypeDef result)
{
    if (i2c_transaction_count >= HAL_MOCK_I2C_TRANSACTION_CAPACITY)
    {
        i2c_transaction_overflow_count++;
        fprintf(stderr, "HAL mock I2C transaction log full (capacity=%u)\n",
                (unsigned)HAL_MOCK_I2C_TRANSACTION_CAPACITY);
        return;
    }

    hal_mock_i2c_transaction_t *entry =
        &i2c_transactions[i2c_transaction_count++];
    entry->handle = hi2c;
    entry->device_address = device_address;
    entry->register_address = register_address;
    entry->register_address_size = register_address_size;
    entry->data_length = data_length;
    entry->timeout = timeout;
    entry->result = result;

    size_t bytes_to_copy = data_length;
    if (bytes_to_copy > HAL_MOCK_I2C_PAYLOAD_CAPACITY)
    {
        bytes_to_copy = HAL_MOCK_I2C_PAYLOAD_CAPACITY;
        i2c_payload_overflow_count++;
        fprintf(stderr, "HAL mock I2C payload exceeds log capacity (length=%u, capacity=%u)\n",
                (unsigned)data_length, (unsigned)HAL_MOCK_I2C_PAYLOAD_CAPACITY);
    }
    if (bytes_to_copy != 0U && data == NULL)
    {
        i2c_payload_overflow_count++;
        fprintf(stderr, "HAL mock I2C payload pointer is null (length=%u)\n",
                (unsigned)data_length);
    }
    else if (bytes_to_copy != 0U)
    {
        memcpy(entry->data, data, bytes_to_copy);
    }
}

void hal_mock_reset(void)
{
    hal_mock_i2c_transactions_reset();
    fake_tick = 0U;
    fake_primask = 0U;
    i2c_write_count = 0U;
    i2c_write_duration_ms = 0U;
    fail_next_i2c_write = 0U;
    fail_i2c_write_countdown = 0U;
    uart_abort_result = HAL_OK;
    uart_start_result = HAL_OK;
    uart_abort_count = 0U;
    uart_start_count = 0U;
    uart_clear_overrun_count = 0U;
}

void hal_mock_uart_clear_overrun(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance != NULL)
    {
        (void)huart->Instance->SR;
        (void)huart->Instance->DR;
        huart->Instance->SR = 0U;
        huart->Instance->DR = 0U;
        uart_clear_overrun_count++;
    }
}

uint32_t hal_mock_uart_clear_overrun_count(void)
{
    return uart_clear_overrun_count;
}

void hal_mock_uart_set_abort_result(HAL_StatusTypeDef result)
{
    uart_abort_result = result;
}

void hal_mock_uart_set_start_result(HAL_StatusTypeDef result)
{
    uart_start_result = result;
}

uint32_t hal_mock_uart_abort_count(void)
{
    return uart_abort_count;
}

uint32_t hal_mock_uart_start_count(void)
{
    return uart_start_count;
}

uint32_t hal_mock_get_primask(void)
{
    return fake_primask;
}

void hal_mock_disable_irq(void)
{
    fake_primask = 1U;
}

void hal_mock_enable_irq(void)
{
    fake_primask = 0U;
}

void hal_mock_set_primask(uint32_t primask)
{
    fake_primask = (primask != 0U) ? 1U : 0U;
}

void hal_mock_set_tick(uint32_t tick)
{
    fake_tick = tick;
}

void hal_mock_advance_tick(uint32_t milliseconds)
{
    fake_tick += milliseconds;
}

void hal_mock_set_i2c_write_duration(uint32_t milliseconds)
{
    i2c_write_duration_ms = milliseconds;
}

uint32_t hal_mock_get_i2c_write_count(void)
{
    return i2c_write_count;
}

uint32_t hal_mock_get_tick(void)
{
    return fake_tick;
}

void hal_mock_fail_next_i2c_write(void)
{
    fail_next_i2c_write = 1U;
}

void hal_mock_fail_i2c_write_on_nth_from_now(uint32_t nth_write)
{
    fail_i2c_write_countdown = nth_write;
}

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

void HAL_Delay(uint32_t Delay)
{
    fake_tick += Delay;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
                                    uint16_t DevAddress,
                                    uint16_t MemAddress,
                                    uint16_t MemAddSize,
                                    uint8_t *pData,
                                    uint16_t Size,
                                    uint32_t Timeout)
{
    i2c_write_count++;
    fake_tick += i2c_write_duration_ms;

    uint8_t should_fail = fail_next_i2c_write;
    fail_next_i2c_write = 0U;

    if (fail_i2c_write_countdown != 0U)
    {
        fail_i2c_write_countdown--;
        if (fail_i2c_write_countdown == 0U)
        {
            should_fail = 1U;
        }
    }

    HAL_StatusTypeDef result = (should_fail != 0U) ? HAL_ERROR : HAL_OK;
    hal_mock_record_i2c_write(hi2c, DevAddress, MemAddress, MemAddSize,
                              pData, Size, Timeout, result);
    return result;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
    uart_abort_count++;

    if (uart_abort_result == HAL_OK)
    {
        huart->RxState = HAL_UART_STATE_READY;
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
        if (huart->hdmarx != NULL)
        {
            huart->hdmarx->State = HAL_DMA_STATE_READY;
        }
    }

    return uart_abort_result;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               uint8_t *pData,
                                               uint16_t Size)
{
    (void)pData;
    (void)Size;
    uart_start_count++;

    if (uart_start_result == HAL_OK)
    {
        huart->RxState = HAL_UART_STATE_BUSY_RX;
        huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
        if (huart->hdmarx != NULL)
        {
            huart->hdmarx->State = HAL_DMA_STATE_BUSY;
        }
    }

    return uart_start_result;
}
