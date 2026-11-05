#ifndef TEST_HOST_STM32F4XX_HAL_H
#define TEST_HOST_STM32F4XX_HAL_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t mock_placeholder;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT 0x00000001U

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
                                    uint16_t DevAddress,
                                    uint16_t MemAddress,
                                    uint16_t MemAddSize,
                                    uint8_t *pData,
                                    uint16_t Size,
                                    uint32_t Timeout);

#endif
