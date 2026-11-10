#include "uart_dma_rx.h"

void uart_dma_rx_init(uart_dma_rx_t *rx)
{
    if (rx != 0)
    {
        rx->last_pos = 0U;
    }
}

void uart_dma_rx_collect(uart_dma_rx_t *rx,
                         const uint8_t *dma_buf,
                         uint16_t pos,
                         uart_line_rx_t *line_rx)
{
    if (rx == 0 || dma_buf == 0 || line_rx == 0)
    {
        return;
    }

    if (pos > UART_DMA_RX_BUF_SIZE)
    {
        uart_line_rx_mark_overflow_isr(line_rx);
        return;
    }

    uint16_t old = rx->last_pos;

    if (pos > old)
    {
        for (uint16_t i = old; i < pos; i++)
        {
            uart_line_rx_push_isr(line_rx, dma_buf[i]);
        }
    }
    else if (pos < old)
    {
        for (uint16_t i = old; i < UART_DMA_RX_BUF_SIZE; i++)
        {
            uart_line_rx_push_isr(line_rx, dma_buf[i]);
        }

        for (uint16_t i = 0U; i < pos; i++)
        {
            uart_line_rx_push_isr(line_rx, dma_buf[i]);
        }
    }

    /* Keep Size == buffer length as the end-of-buffer position. */
    rx->last_pos = pos;
}
