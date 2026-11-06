#include "stm32f4xx_hal.h"

static uint32_t fake_tick;
static uint32_t i2c_write_count;
static uint32_t i2c_write_duration_ms;
static uint8_t fail_next_i2c_write;

void hal_mock_reset(void)
{
    fake_tick = 0U;
    i2c_write_count = 0U;
    i2c_write_duration_ms = 0U;
    fail_next_i2c_write = 0U;
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
