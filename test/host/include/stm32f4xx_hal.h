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

typedef enum
{
    HAL_UART_STATE_RESET = 0x00U,
    HAL_UART_STATE_READY = 0x20U,
    HAL_UART_STATE_BUSY_RX = 0x22U
} HAL_UART_StateTypeDef;

typedef enum
{
    HAL_DMA_STATE_RESET = 0x00U,
    HAL_DMA_STATE_READY = 0x01U,
    HAL_DMA_STATE_BUSY = 0x02U,
    HAL_DMA_STATE_ERROR = 0x04U,
    HAL_DMA_STATE_ABORT = 0x05U
} HAL_DMA_StateTypeDef;

typedef struct
{
    volatile uint32_t SR;
    volatile uint32_t DR;
} USART_TypeDef;

#define HAL_UART_RECEPTION_STANDARD 0x00000000U
#define HAL_UART_RECEPTION_TOIDLE   0x00000001U

#define HAL_UART_ERROR_PE           0x00000001U
#define HAL_UART_ERROR_NE           0x00000002U
#define HAL_UART_ERROR_FE           0x00000004U
#define HAL_UART_ERROR_ORE          0x00000008U
#define HAL_UART_ERROR_DMA          0x00000010U

typedef struct
{
    HAL_DMA_StateTypeDef State;
} DMA_HandleTypeDef;

typedef struct
{
    USART_TypeDef *Instance;
    HAL_UART_StateTypeDef RxState;
    uint32_t ReceptionType;
    uint32_t ErrorCode;
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT 0x00000001U

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t hal_mock_get_primask(void);
void hal_mock_disable_irq(void);
void hal_mock_enable_irq(void);

#define __get_PRIMASK() hal_mock_get_primask()
#define __disable_irq() hal_mock_disable_irq()
#define __enable_irq() hal_mock_enable_irq()

void hal_mock_uart_clear_overrun(UART_HandleTypeDef *huart);
#define __HAL_UART_CLEAR_OREFLAG(__HANDLE__) \
    hal_mock_uart_clear_overrun((__HANDLE__))

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
                                    uint16_t DevAddress,
                                    uint16_t MemAddress,
                                    uint16_t MemAddSize,
                                    uint8_t *pData,
                                    uint16_t Size,
                                    uint32_t Timeout);

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               uint8_t *pData,
                                               uint16_t Size);

#endif
