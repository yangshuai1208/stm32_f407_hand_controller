#include "main.h"
#include "uart_rx_recovery.h"

static uint32_t uart_rx_recovery_saturating_increment(volatile uint32_t *value)
{
    if (*value < UINT32_MAX)
    {
        (*value)++;
    }

    return *value;
}

static uint8_t uart_rx_recovery_deadline_reached(uint32_t now,
                                                  uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static uint32_t uart_rx_recovery_backoff(uint8_t attempts)
{
    uint32_t delay = UART_RX_RECOVERY_RETRY_INITIAL_MS;

    for (uint8_t i = 1U; i < attempts; i++)
    {
        if (delay >= UART_RX_RECOVERY_RETRY_MAX_MS / 2U)
        {
            return UART_RX_RECOVERY_RETRY_MAX_MS;
        }

        delay *= 2U;
    }

    return (delay > UART_RX_RECOVERY_RETRY_MAX_MS)
        ? UART_RX_RECOVERY_RETRY_MAX_MS
        : delay;
}

static void uart_rx_recovery_fail_attempt(
    uart_rx_recovery_t *recovery,
    uart_rx_recovery_failure_t reason,
    uint32_t now)
{
    recovery->last_failure = reason;

    if (recovery->attempts >= UART_RX_RECOVERY_MAX_ATTEMPTS)
    {
        recovery->state = UART_RX_RECOVERY_FAULT;
        recovery->last_failure = UART_RX_FAILURE_RETRY_LIMIT;
        return;
    }

    recovery->retry_deadline =
        now + uart_rx_recovery_backoff(recovery->attempts);
    recovery->state = UART_RX_RECOVERY_BACKOFF;
}

void uart_rx_recovery_init(uart_rx_recovery_t *recovery)
{
    if (recovery == 0)
    {
        return;
    }

    recovery->pending_error_code = 0U;
    recovery->total_error_count = 0U;
    recovery->ore_count = 0U;
    recovery->fe_count = 0U;
    recovery->ne_count = 0U;
    recovery->pe_count = 0U;
    recovery->dma_error_count = 0U;
    recovery->error_pending = 0U;
    recovery->state = UART_RX_RECOVERY_ACTIVE;
    recovery->last_failure = UART_RX_FAILURE_NONE;
    recovery->attempts = 0U;
    recovery->state_tick = 0U;
    recovery->retry_deadline = 0U;
}

void uart_rx_recovery_mark_started(uart_rx_recovery_t *recovery)
{
    if (recovery != 0 && recovery->error_pending == 0U)
    {
        recovery->state = UART_RX_RECOVERY_ACTIVE;
        recovery->last_failure = UART_RX_FAILURE_NONE;
    }
}

void uart_rx_recovery_on_error(uart_rx_recovery_t *recovery,
                               UART_HandleTypeDef *huart,
                               UART_HandleTypeDef *usart1)
{
    if (recovery == 0 || huart == 0 || huart != usart1)
    {
        return;
    }

    uint32_t error_code = huart->ErrorCode;
    if (error_code == 0U)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    recovery->pending_error_code |= error_code;
    recovery->error_pending = 1U;
    (void)uart_rx_recovery_saturating_increment(
        &recovery->total_error_count);

    if ((error_code & HAL_UART_ERROR_ORE) != 0U)
    {
        (void)uart_rx_recovery_saturating_increment(&recovery->ore_count);
    }
    if ((error_code & HAL_UART_ERROR_FE) != 0U)
    {
        (void)uart_rx_recovery_saturating_increment(&recovery->fe_count);
    }
    if ((error_code & HAL_UART_ERROR_NE) != 0U)
    {
        (void)uart_rx_recovery_saturating_increment(&recovery->ne_count);
    }
    if ((error_code & HAL_UART_ERROR_PE) != 0U)
    {
        (void)uart_rx_recovery_saturating_increment(&recovery->pe_count);
    }
    if ((error_code & HAL_UART_ERROR_DMA) != 0U)
    {
        (void)uart_rx_recovery_saturating_increment(
            &recovery->dma_error_count);
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint32_t uart_rx_recovery_take_error(uart_rx_recovery_t *recovery)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t error_code = recovery->pending_error_code;
    recovery->pending_error_code = 0U;
    recovery->error_pending = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return error_code;
}

static uint8_t uart_rx_recovery_receive_is_active(
    const UART_HandleTypeDef *huart)
{
    return huart->RxState == HAL_UART_STATE_BUSY_RX &&
           huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE &&
           huart->hdmarx != 0 &&
           huart->hdmarx->State == HAL_DMA_STATE_BUSY;
}

void uart_rx_recovery_service(uart_rx_recovery_t *recovery,
                              UART_HandleTypeDef *huart,
                              uint8_t *dma_buffer,
                              uart_dma_rx_t *dma_rx,
                              uart_line_rx_t *line_rx)
{
    if (recovery == 0 || huart == 0 || dma_buffer == 0 ||
        dma_rx == 0 || line_rx == 0 || huart->hdmarx == 0)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t error_code = uart_rx_recovery_take_error(recovery);

    if (error_code != 0U)
    {
        if (recovery->state == UART_RX_RECOVERY_ACTIVE)
        {
            recovery->state = UART_RX_RECOVERY_WAIT_DMA;
            recovery->state_tick = now;
            recovery->attempts = 0U;
        }
    }

    if (recovery->state == UART_RX_RECOVERY_ACTIVE ||
        recovery->state == UART_RX_RECOVERY_FAULT)
    {
        return;
    }

    if (recovery->state == UART_RX_RECOVERY_BACKOFF)
    {
        if (!uart_rx_recovery_deadline_reached(
                now,
                recovery->retry_deadline))
        {
            return;
        }

        recovery->state = UART_RX_RECOVERY_WAIT_DMA;
        recovery->state_tick = now;
    }

    /* Recoverable HAL notifications can occur while the circular RX is valid. */
    if (uart_rx_recovery_receive_is_active(huart))
    {
        recovery->state = UART_RX_RECOVERY_ACTIVE;
        recovery->attempts = 0U;
        recovery->last_failure = UART_RX_FAILURE_NONE;
        return;
    }

    if (huart->hdmarx->State != HAL_DMA_STATE_READY)
    {
        if ((uint32_t)(now - recovery->state_tick) >=
            UART_RX_RECOVERY_DMA_WAIT_TIMEOUT_MS)
        {
            recovery->state = UART_RX_RECOVERY_FAULT;
            recovery->last_failure = UART_RX_FAILURE_DMA_NOT_READY;
        }
        return;
    }

    if (recovery->attempts >= UART_RX_RECOVERY_MAX_ATTEMPTS)
    {
        recovery->state = UART_RX_RECOVERY_FAULT;
        recovery->last_failure = UART_RX_FAILURE_RETRY_LIMIT;
        return;
    }

    recovery->attempts++;

    HAL_StatusTypeDef abort_status = HAL_UART_AbortReceive(huart);
    if (abort_status != HAL_OK)
    {
        uart_rx_recovery_fail_attempt(
            recovery,
            UART_RX_FAILURE_ABORT,
            now);
        return;
    }

    if (huart->RxState != HAL_UART_STATE_READY ||
        huart->ReceptionType != HAL_UART_RECEPTION_STANDARD ||
        huart->hdmarx->State != HAL_DMA_STATE_READY)
    {
        uart_rx_recovery_fail_attempt(
            recovery,
            UART_RX_FAILURE_ABORT_STATE,
            now);
        return;
    }

    /*
     * HAL_UART_AbortReceive() stops DMA but does not drain USART DR.
     * Clear the SR/DR error sequence before opening a fresh software frame.
     */
    __HAL_UART_CLEAR_OREFLAG(huart);
    uart_line_rx_reset_after_uart_abort(line_rx);
    uart_dma_rx_init(dma_rx);

    HAL_StatusTypeDef start_status = HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        dma_buffer,
        UART_DMA_RX_BUF_SIZE);

    if (uart_rx_recovery_receive_is_active(huart))
    {
        recovery->state = UART_RX_RECOVERY_ACTIVE;
        recovery->attempts = 0U;
        recovery->last_failure = UART_RX_FAILURE_NONE;
        return;
    }

    if (start_status != HAL_OK)
    {
        uart_rx_recovery_fail_attempt(
            recovery,
            UART_RX_FAILURE_START,
            now);
        return;
    }

    uart_rx_recovery_fail_attempt(
        recovery,
        UART_RX_FAILURE_START,
        now);
}

uint8_t uart_rx_recovery_commands_allowed(
    const uart_rx_recovery_t *recovery)
{
    return recovery != 0 &&
           recovery->state == UART_RX_RECOVERY_ACTIVE &&
           recovery->error_pending == 0U;
}
