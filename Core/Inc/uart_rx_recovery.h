#ifndef UART_RX_RECOVERY_H
#define UART_RX_RECOVERY_H

#include <stdint.h>

#include "main.h"
#include "uart_dma_rx.h"
#include "uart_line.h"

#define UART_RX_RECOVERY_MAX_ATTEMPTS          5U
#define UART_RX_RECOVERY_RETRY_INITIAL_MS     50U
#define UART_RX_RECOVERY_RETRY_MAX_MS        800U
#define UART_RX_RECOVERY_DMA_WAIT_TIMEOUT_MS 100U

typedef enum
{
    UART_RX_RECOVERY_ACTIVE = 0U,
    UART_RX_RECOVERY_WAIT_DMA,
    UART_RX_RECOVERY_BACKOFF,
    UART_RX_RECOVERY_FAULT
} uart_rx_recovery_state_t;

typedef enum
{
    UART_RX_FAILURE_NONE = 0U,
    UART_RX_FAILURE_DMA_NOT_READY,
    UART_RX_FAILURE_ABORT,
    UART_RX_FAILURE_ABORT_STATE,
    UART_RX_FAILURE_START,
    UART_RX_FAILURE_RETRY_LIMIT
} uart_rx_recovery_failure_t;

typedef struct
{
    volatile uint32_t pending_error_code;
    volatile uint32_t total_error_count;
    volatile uint32_t ore_count;
    volatile uint32_t fe_count;
    volatile uint32_t ne_count;
    volatile uint32_t pe_count;
    volatile uint32_t dma_error_count;
    volatile uint8_t error_pending;

    uart_rx_recovery_state_t state;
    uart_rx_recovery_failure_t last_failure;
    uint8_t attempts;
    uint32_t state_tick;
    uint32_t retry_deadline;
} uart_rx_recovery_t;

void uart_rx_recovery_init(uart_rx_recovery_t *recovery);
void uart_rx_recovery_mark_started(uart_rx_recovery_t *recovery);
void uart_rx_recovery_on_error(uart_rx_recovery_t *recovery,
                               UART_HandleTypeDef *huart,
                               UART_HandleTypeDef *usart1);
void uart_rx_recovery_service(uart_rx_recovery_t *recovery,
                              UART_HandleTypeDef *huart,
                              uint8_t *dma_buffer,
                              uart_dma_rx_t *dma_rx,
                              uart_line_rx_t *line_rx);
uint8_t uart_rx_recovery_commands_allowed(
    const uart_rx_recovery_t *recovery);

#endif
