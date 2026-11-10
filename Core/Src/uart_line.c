#include "main.h"
#include "uart_line.h"

void uart_line_rx_init(uart_line_rx_t *rx)
{
    if (rx == NULL)
    {
        return;
    }

    rx->head = 0U;
    rx->tail = 0U;
    rx->overflow = 0U;
    rx->line_index = 0U;
    rx->line_discard = 0U;
}

void uart_line_rx_push_isr(uart_line_rx_t *rx, uint8_t byte)
{
    if (rx == NULL || rx->overflow != 0U)
    {
        return;
    }

    uint16_t next =
        (uint16_t)((rx->head + 1U) % UART_LINE_RING_SIZE);

    if (next == rx->tail)
    {
        rx->overflow = 1U;
        return;
    }

    rx->ring[rx->head] = byte;
    rx->head = next;
}

void uart_line_rx_mark_overflow_isr(uart_line_rx_t *rx)
{
    if (rx != NULL)
    {
        rx->overflow = 1U;
    }
}

uint8_t uart_line_rx_has_overflow(const uart_line_rx_t *rx)
{
    return (rx != NULL) ? rx->overflow : 0U;
}

void uart_line_rx_recover_overflow(uart_line_rx_t *rx)
{
    if (rx == NULL)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (rx->overflow != 0U)
    {
        rx->tail = rx->head;
        rx->overflow = 0U;
        rx->line_index = 0U;
        rx->line_discard = 1U;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void uart_line_rx_force_resync(uart_line_rx_t *rx)
{
    if (rx == NULL)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    rx->tail = rx->head;
    rx->overflow = 0U;
    rx->line_index = 0U;
    rx->line_discard = 1U;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void uart_line_rx_reset_after_uart_abort(uart_line_rx_t *rx)
{
    if (rx == NULL)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    rx->tail = rx->head;
    rx->overflow = 0U;
    rx->line_index = 0U;
    rx->line_discard = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t uart_line_rx_pop(uart_line_rx_t *rx, uint8_t *byte)
{
    if (rx == NULL || byte == NULL)
    {
        return 0U;
    }

    uint8_t has_data = 0U;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (rx->tail != rx->head)
    {
        *byte = rx->ring[rx->tail];
        rx->tail =
            (uint16_t)((rx->tail + 1U) % UART_LINE_RING_SIZE);
        has_data = 1U;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return has_data;
}

uint8_t uart_line_rx_poll(uart_line_rx_t *rx,
                          char out[UART_LINE_BUFFER_SIZE])
{
    if (rx == NULL || out == NULL)
    {
        return 0U;
    }

    uint8_t byte;
    uint16_t consumed = 0U;

    while (consumed < UART_LINE_POLL_BUDGET)
    {
        if (!uart_line_rx_pop(rx, &byte))
        {
            return 0U;
        }

        consumed++;

        if (byte == '\n')
        {
            uint8_t valid =
                !rx->line_discard &&
                rx->line_index > 0U;

            if (valid)
            {
                rx->line_buf[rx->line_index] = '\0';
                for (uint8_t i = 0U; i <= rx->line_index; i++)
                {
                    out[i] = rx->line_buf[i];
                }
            }

            rx->line_index = 0U;
            rx->line_discard = 0U;
            return valid;
        }

        if (byte == '\r')
        {
            continue;
        }

        if (!rx->line_discard)
        {
            if (rx->line_index < UART_LINE_BUFFER_SIZE - 1U)
            {
                rx->line_buf[rx->line_index++] = (char)byte;
            }
            else
            {
                rx->line_discard = 1U;
            }
        }
    }

    return 0U;
}

#ifdef UART_LINE_TEST
uint16_t uart_line_rx_pending_for_test(const uart_line_rx_t *rx)
{
    if (rx == NULL)
    {
        return 0U;
    }

    if (rx->head >= rx->tail)
    {
        return (uint16_t)(rx->head - rx->tail);
    }

    return (uint16_t)(UART_LINE_RING_SIZE - rx->tail + rx->head);
}
#endif
