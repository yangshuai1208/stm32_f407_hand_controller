#ifndef UART_DMA_RX_H
#define UART_DMA_RX_H

#include <stdint.h>

#include "uart_line.h"

#define UART_DMA_RX_BUF_SIZE 256U

typedef struct
{
    uint16_t last_pos;
} uart_dma_rx_t;

void uart_dma_rx_init(uart_dma_rx_t *rx);
void uart_dma_rx_collect(uart_dma_rx_t *rx,
                         const uint8_t *dma_buf,
                         uint16_t pos,
                         uart_line_rx_t *line_rx);

#endif
