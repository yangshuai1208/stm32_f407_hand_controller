#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "uart_line.h"

void hal_mock_reset(void);
uint32_t hal_mock_get_primask(void);
void hal_mock_set_primask(uint32_t primask);

static uart_line_rx_t rx;

static void reset_fixture(void)
{
    hal_mock_reset();
    uart_line_rx_init(&rx);
}

static void push_text(const char *text)
{
    while (*text != '\0')
    {
        uart_line_rx_push_isr(&rx, (uint8_t)*text);
        text++;
    }
}

static void expect_line(const char *expected)
{
    char line[UART_LINE_BUFFER_SIZE];

    assert(uart_line_rx_poll(&rx, line) == 1U);
    assert(strcmp(line, expected) == 0);
}

static void test_complete_frame(void)
{
    reset_fixture();
    push_text("SEQ:7 CMD:HAND_STOP\n");
    expect_line("SEQ:7 CMD:HAND_STOP");
    assert(uart_line_rx_pending_for_test(&rx) == 0U);
    assert(hal_mock_get_primask() == 0U);
}

static void test_half_frame_across_polls(void)
{
    reset_fixture();
    char line[UART_LINE_BUFFER_SIZE] = "unchanged";

    push_text("SEQ:8 CMD:HAND_");
    assert(uart_line_rx_poll(&rx, line) == 0U);
    assert(strcmp(line, "unchanged") == 0);

    push_text("STOP\n");
    expect_line("SEQ:8 CMD:HAND_STOP");
}

static void test_two_frames_are_returned_separately(void)
{
    reset_fixture();
    push_text("FIRST\nSECOND\n");

    expect_line("FIRST");
    assert(uart_line_rx_pending_for_test(&rx) == 7U);
    expect_line("SECOND");
    assert(uart_line_rx_pending_for_test(&rx) == 0U);
}

static void test_crlf(void)
{
    reset_fixture();
    push_text("CRLF_FRAME\r\n");
    expect_line("CRLF_FRAME");
}

static void test_maximum_length_frame(void)
{
    reset_fixture();
    char frame[UART_LINE_BUFFER_SIZE];

    memset(frame, 'M', UART_LINE_BUFFER_SIZE - 1U);
    frame[UART_LINE_BUFFER_SIZE - 1U] = '\0';
    push_text(frame);
    push_text("\n");
    expect_line(frame);
}

static void test_overlong_frame_recovers_at_next_frame(void)
{
    reset_fixture();
    char overlong[UART_LINE_BUFFER_SIZE + 1U];

    memset(overlong, 'X', UART_LINE_BUFFER_SIZE);
    overlong[UART_LINE_BUFFER_SIZE] = '\0';
    push_text(overlong);
    push_text("\n");

    char line[UART_LINE_BUFFER_SIZE];
    assert(uart_line_rx_poll(&rx, line) == 0U);
    assert(uart_line_rx_pending_for_test(&rx) == 0U);

    push_text("RECOVERED\n");
    expect_line("RECOVERED");
}

static void test_poll_consumes_at_most_65_successful_pops(void)
{
    reset_fixture();

    for (uint16_t i = 0U; i < UART_LINE_POLL_BUDGET + 1U; i++)
    {
        uart_line_rx_push_isr(&rx, (uint8_t)'Z');
    }

    char line[UART_LINE_BUFFER_SIZE];
    assert(uart_line_rx_poll(&rx, line) == 0U);
    assert(uart_line_rx_pending_for_test(&rx) == 1U);

    assert(uart_line_rx_poll(&rx, line) == 0U);
    assert(uart_line_rx_pending_for_test(&rx) == 0U);

    push_text("\nBUDGET_RECOVERED\n");
    assert(uart_line_rx_poll(&rx, line) == 0U);
    expect_line("BUDGET_RECOVERED");
}

static void test_ring_index_wraparound(void)
{
    reset_fixture();

    /* Move both indices to 510 without leaving line data pending. */
    for (uint16_t i = 0U; i < UART_LINE_RING_SIZE - 2U; i++)
    {
        uart_line_rx_push_isr(&rx, (uint8_t)'\r');
    }

    for (uint8_t i = 0U; i < 8U; i++)
    {
        char line[UART_LINE_BUFFER_SIZE];
        assert(uart_line_rx_poll(&rx, line) == 0U);
    }
    assert(uart_line_rx_pending_for_test(&rx) == 0U);

    /* XYZ\n crosses the ring's final slot and wraps to index zero. */
    push_text("XYZ\n");
    expect_line("XYZ");
}

static void test_overflow_flush_and_discard_recovery(void)
{
    reset_fixture();
    push_text("PART");
    char line[UART_LINE_BUFFER_SIZE];
    assert(uart_line_rx_poll(&rx, line) == 0U);

    for (uint16_t i = 0U; i < UART_LINE_RING_SIZE; i++)
    {
        uart_line_rx_push_isr(&rx, (uint8_t)'Q');
    }
    assert(uart_line_rx_has_overflow(&rx) == 1U);
    assert(uart_line_rx_pending_for_test(&rx) == UART_LINE_RING_SIZE - 1U);

    /* Recovery must not enable interrupts that were already masked. */
    hal_mock_set_primask(1U);
    uart_line_rx_recover_overflow(&rx);
    assert(hal_mock_get_primask() == 1U);
    hal_mock_set_primask(0U);

    assert(uart_line_rx_has_overflow(&rx) == 0U);
    assert(uart_line_rx_pending_for_test(&rx) == 0U);

    push_text("\nAFTER_OVERFLOW\n");
    assert(uart_line_rx_poll(&rx, line) == 0U);
    expect_line("AFTER_OVERFLOW");
}

static void test_pop_preserves_preexisting_irq_mask(void)
{
    reset_fixture();
    push_text("MASK\n");

    hal_mock_set_primask(1U);
    expect_line("MASK");
    assert(hal_mock_get_primask() == 1U);
}

int main(void)
{
    test_complete_frame();
    test_half_frame_across_polls();
    test_two_frames_are_returned_separately();
    test_crlf();
    test_maximum_length_frame();
    test_overlong_frame_recovers_at_next_frame();
    test_poll_consumes_at_most_65_successful_pops();
    test_ring_index_wraparound();
    test_overflow_flush_and_discard_recovery();
    test_pop_preserves_preexisting_irq_mask();

    return 0;
}
