#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "uart_dma_rx.h"

void hal_mock_reset(void);

static uart_dma_rx_t dma_rx;
static uart_line_rx_t line_rx;
static uint8_t guarded_dma[UART_DMA_RX_BUF_SIZE + 2U];

static uint8_t *dma_buf(void)
{
    return &guarded_dma[1];
}

static void reset_fixture(void)
{
    hal_mock_reset();
    uart_dma_rx_init(&dma_rx);
    uart_line_rx_init(&line_rx);
    memset(guarded_dma, 0xA5, sizeof(guarded_dma));
}

static uint16_t queue_count(void)
{
    return uart_line_rx_pending_for_test(&line_rx);
}

static void assert_queue_bytes(const uint8_t *expected, uint16_t count)
{
    assert(queue_count() == count);

    uint16_t index = line_rx.tail;
    for (uint16_t i = 0U; i < count; i++)
    {
        assert(line_rx.ring[index] == expected[i]);
        index = (uint16_t)((index + 1U) % UART_LINE_RING_SIZE);
    }
}

static void fill_pattern(uint16_t begin, uint16_t end, uint8_t salt)
{
    for (uint16_t i = begin; i < end; i++)
    {
        dma_buf()[i] = (uint8_t)(i + salt);
    }
}

static void assert_dma_guards(void)
{
    assert(guarded_dma[0] == 0xA5U);
    assert(guarded_dma[UART_DMA_RX_BUF_SIZE + 1U] == 0xA5U);
}

static void test_128_256_256_3(void)
{
    uint8_t expected[UART_DMA_RX_BUF_SIZE + 3U];
    reset_fixture();

    fill_pattern(0U, UART_DMA_RX_BUF_SIZE, 0U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 128U, &line_rx);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 256U, &line_rx);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 256U, &line_rx);

    fill_pattern(0U, 3U, 0xE0U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 3U, &line_rx);

    for (uint16_t i = 0U; i < 256U; i++)
    {
        expected[i] = (uint8_t)i;
    }
    expected[256] = 0xE0U;
    expected[257] = 0xE1U;
    expected[258] = 0xE2U;

    assert_queue_bytes(expected, 259U);
    assert_dma_guards();
}

static void test_128_200_256_4(void)
{
    uint8_t expected[UART_DMA_RX_BUF_SIZE + 4U];
    reset_fixture();

    fill_pattern(0U, 256U, 0U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 128U, &line_rx);
    fill_pattern(128U, 200U, 0x20U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 200U, &line_rx);
    fill_pattern(200U, 256U, 0x40U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 256U, &line_rx);
    fill_pattern(0U, 4U, 0xD0U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 4U, &line_rx);

    for (uint16_t i = 0U; i < 128U; i++)
    {
        expected[i] = (uint8_t)i;
    }
    for (uint16_t i = 128U; i < 200U; i++)
    {
        expected[i] = (uint8_t)(i + 0x20U);
    }
    for (uint16_t i = 200U; i < 256U; i++)
    {
        expected[i] = (uint8_t)(i + 0x40U);
    }
    expected[256] = 0xD0U;
    expected[257] = 0xD1U;
    expected[258] = 0xD2U;
    expected[259] = 0xD3U;

    assert_queue_bytes(expected, 260U);
    assert_dma_guards();
}

static void test_same_non_boundary_position_is_ignored(void)
{
    uint8_t expected[32];
    reset_fixture();
    fill_pattern(0U, 32U, 0x10U);

    uart_dma_rx_collect(&dma_rx, dma_buf(), 32U, &line_rx);
    memset(dma_buf(), 0xEE, 32U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 32U, &line_rx);

    for (uint16_t i = 0U; i < 32U; i++)
    {
        expected[i] = (uint8_t)(i + 0x10U);
    }
    assert_queue_bytes(expected, 32U);
    assert_dma_guards();
}

static void test_full_buffer_then_normal_wrap(void)
{
    uint8_t expected[UART_DMA_RX_BUF_SIZE + 5U];
    reset_fixture();

    fill_pattern(0U, 256U, 0U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 256U, &line_rx);
    fill_pattern(0U, 5U, 0xC0U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 5U, &line_rx);

    for (uint16_t i = 0U; i < 256U; i++)
    {
        expected[i] = (uint8_t)i;
    }
    for (uint16_t i = 0U; i < 5U; i++)
    {
        expected[256U + i] = (uint8_t)(0xC0U + i);
    }

    assert_queue_bytes(expected, 261U);
    assert_dma_guards();
}

static void test_invalid_size_marks_overflow_without_copy(void)
{
    uint8_t expected[8];
    reset_fixture();
    fill_pattern(0U, 8U, 0x30U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 8U, &line_rx);

    uart_dma_rx_collect(&dma_rx, dma_buf(), 257U, &line_rx);

    for (uint16_t i = 0U; i < 8U; i++)
    {
        expected[i] = (uint8_t)(i + 0x30U);
    }
    assert_queue_bytes(expected, 8U);
    assert(uart_line_rx_has_overflow(&line_rx) == 1U);
    assert(dma_rx.last_pos == 8U);
    assert_dma_guards();

    uart_line_rx_recover_overflow(&line_rx);
    assert(uart_line_rx_has_overflow(&line_rx) == 0U);
    assert(queue_count() == 0U);

    uart_line_rx_push_isr(&line_rx, (uint8_t)'\n');
    uart_line_rx_push_isr(&line_rx, (uint8_t)'O');
    uart_line_rx_push_isr(&line_rx, (uint8_t)'K');
    uart_line_rx_push_isr(&line_rx, (uint8_t)'\n');
    char line[UART_LINE_BUFFER_SIZE];
    assert(uart_line_rx_poll(&line_rx, line) == 0U);
    assert(uart_line_rx_poll(&line_rx, line) == 1U);
    assert(strcmp(line, "OK") == 0);
}

static void test_software_queue_overflow_recovery(void)
{
    uint8_t expected[UART_LINE_RING_SIZE - 1U];
    reset_fixture();

    fill_pattern(0U, 256U, 0U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 256U, &line_rx);

    fill_pattern(0U, 128U, 0x20U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 128U, &line_rx);
    fill_pattern(128U, 256U, 0x40U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 256U, &line_rx);

    fill_pattern(0U, 128U, 0x60U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 128U, &line_rx);
    fill_pattern(128U, 256U, 0x80U);
    uart_dma_rx_collect(&dma_rx, dma_buf(), 256U, &line_rx);

    for (uint16_t i = 0U; i < 256U; i++)
    {
        expected[i] = (uint8_t)i;
    }
    for (uint16_t i = 0U; i < 128U; i++)
    {
        expected[256U + i] = (uint8_t)(i + 0x20U);
    }
    for (uint16_t i = 0U; i < 127U; i++)
    {
        expected[384U + i] = (uint8_t)(i + 128U + 0x40U);
    }

    assert_queue_bytes(expected, UART_LINE_RING_SIZE - 1U);
    assert(uart_line_rx_has_overflow(&line_rx) == 1U);
    assert_dma_guards();

    uart_line_rx_recover_overflow(&line_rx);
    assert(queue_count() == 0U);
    assert(uart_line_rx_has_overflow(&line_rx) == 0U);

    uart_line_rx_push_isr(&line_rx, (uint8_t)'\n');
    uart_line_rx_push_isr(&line_rx, (uint8_t)'R');
    uart_line_rx_push_isr(&line_rx, (uint8_t)'X');
    uart_line_rx_push_isr(&line_rx, (uint8_t)'\n');
    char line[UART_LINE_BUFFER_SIZE];
    assert(uart_line_rx_poll(&line_rx, line) == 0U);
    assert(uart_line_rx_poll(&line_rx, line) == 1U);
    assert(strcmp(line, "RX") == 0);
}

int main(void)
{
    test_128_256_256_3();
    test_128_200_256_4();
    test_same_non_boundary_position_is_ignored();
    test_full_buffer_then_normal_wrap();
    test_invalid_size_marks_overflow_without_copy();
    test_software_queue_overflow_recovery();
    return 0;
}
