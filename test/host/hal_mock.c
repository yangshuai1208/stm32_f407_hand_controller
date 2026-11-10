#include "stm32f4xx_hal.h"

static uint32_t fake_tick;
static uint32_t fake_primask;
static uint32_t i2c_write_count;
static uint32_t i2c_write_duration_ms;
static uint8_t fail_next_i2c_write;
static HAL_StatusTypeDef uart_abort_result;
static HAL_StatusTypeDef uart_start_result;
static uint32_t uart_abort_count;
static uint32_t uart_start_count;
static uint32_t uart_clear_overrun_count;

void hal_mock_reset(void)
{
    fake_tick = 0U;
    fake_primask = 0U;
    i2c_write_count = 0U;
    i2c_write_duration_ms = 0U;
    fail_next_i2c_write = 0U;
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
    (void)hi2c;
    (void)DevAddress;
    (void)MemAddress;
    (void)MemAddSize;
    (void)pData;
    (void)Size;
    (void)Timeout;

    i2c_write_count++;
    fake_tick += i2c_write_duration_ms;

    if (fail_next_i2c_write != 0U)
    {
        fail_next_i2c_write = 0U;
        return HAL_ERROR;
    }

    return HAL_OK;
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
