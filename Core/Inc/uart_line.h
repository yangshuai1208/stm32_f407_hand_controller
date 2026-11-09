#ifndef UART_LINE_H
#define UART_LINE_H

#include <stdint.h>

#define UART_LINE_RING_SIZE       512U
#define UART_LINE_BUFFER_SIZE      64U
#define UART_LINE_POLL_BUDGET     (UART_LINE_BUFFER_SIZE + 1U)

typedef struct
{
    uint8_t ring[UART_LINE_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint8_t overflow;

    char line_buf[UART_LINE_BUFFER_SIZE];
    uint8_t line_index;
    uint8_t line_discard;
} uart_line_rx_t;

void uart_line_rx_init(uart_line_rx_t *rx);
void uart_line_rx_push_isr(uart_line_rx_t *rx, uint8_t byte);
void uart_line_rx_mark_overflow_isr(uart_line_rx_t *rx);
uint8_t uart_line_rx_has_overflow(const uart_line_rx_t *rx);
void uart_line_rx_recover_overflow(uart_line_rx_t *rx);
uint8_t uart_line_rx_poll(uart_line_rx_t *rx,
                          char out[UART_LINE_BUFFER_SIZE]);

#ifdef UART_LINE_TEST
uint16_t uart_line_rx_pending_for_test(const uart_line_rx_t *rx);
#endif

#endif
