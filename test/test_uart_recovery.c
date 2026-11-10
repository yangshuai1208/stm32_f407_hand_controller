#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "hand_protocol.h"
#include "uart_rx_recovery.h"

void hal_mock_reset(void);
void hal_mock_advance_tick(uint32_t milliseconds);
void hal_mock_uart_set_abort_result(HAL_StatusTypeDef result);
void hal_mock_uart_set_start_result(HAL_StatusTypeDef result);
uint32_t hal_mock_uart_abort_count(void);
uint32_t hal_mock_uart_start_count(void);
uint32_t hal_mock_uart_clear_overrun_count(void);
uint32_t hal_mock_get_primask(void);
void hal_mock_set_primask(uint32_t primask);

static uart_rx_recovery_t recovery;
static UART_HandleTypeDef uart;
static DMA_HandleTypeDef dma;
static USART_TypeDef usart;
static uart_dma_rx_t dma_rx;
static uart_line_rx_t line_rx;
static uint8_t dma_buffer[UART_DMA_RX_BUF_SIZE];

static void reset_fixture(void)
{
    hal_mock_reset();
    memset(&uart, 0, sizeof(uart));
    memset(&dma, 0, sizeof(dma));
    memset(&usart, 0, sizeof(usart));
    memset(dma_buffer, 0, sizeof(dma_buffer));
    uart.Instance = &usart;
    uart.RxState = HAL_UART_STATE_READY;
    uart.ReceptionType = HAL_UART_RECEPTION_STANDARD;
    uart.hdmarx = &dma;
    dma.State = HAL_DMA_STATE_READY;
    uart_rx_recovery_init(&recovery);
    uart_dma_rx_init(&dma_rx);
    uart_line_rx_init(&line_rx);
}

static void report_error(uint32_t error_code)
{
    uart.ErrorCode = error_code;
    uart_rx_recovery_on_error(&recovery, &uart, &uart);
}

static void service_recovery(void)
{
    uart_rx_recovery_service(
        &recovery,
        &uart,
        dma_buffer,
        &dma_rx,
        &line_rx);
}

static void test_ore_is_recorded_without_hal_work_in_callback(void)
{
    reset_fixture();
    report_error(HAL_UART_ERROR_ORE);

    assert(recovery.error_pending == 1U);
    assert(recovery.pending_error_code == HAL_UART_ERROR_ORE);
    assert(recovery.total_error_count == 1U);
    assert(recovery.ore_count == 1U);
    assert(hal_mock_uart_abort_count() == 0U);
    assert(hal_mock_uart_start_count() == 0U);
}

static void test_fe_ne_and_dma_error_counts(void)
{
    reset_fixture();
    report_error(HAL_UART_ERROR_FE | HAL_UART_ERROR_NE |
                 HAL_UART_ERROR_DMA | HAL_UART_ERROR_PE);

    assert(recovery.total_error_count == 1U);
    assert(recovery.fe_count == 1U);
    assert(recovery.ne_count == 1U);
    assert(recovery.dma_error_count == 1U);
    assert(recovery.pe_count == 1U);
    assert(recovery.pending_error_code ==
           (HAL_UART_ERROR_FE | HAL_UART_ERROR_NE |
            HAL_UART_ERROR_DMA | HAL_UART_ERROR_PE));
    assert(hal_mock_uart_abort_count() == 0U);
    assert(hal_mock_uart_start_count() == 0U);
}

static void test_non_usart1_error_is_ignored(void)
{
    UART_HandleTypeDef other_uart;
    reset_fixture();
    memset(&other_uart, 0, sizeof(other_uart));
    other_uart.ErrorCode = HAL_UART_ERROR_ORE;

    uart_rx_recovery_on_error(&recovery, &other_uart, &uart);

    assert(recovery.total_error_count == 0U);
    assert(recovery.error_pending == 0U);
}

static void test_dma_abort_in_progress_blocks_restart(void)
{
    reset_fixture();
    report_error(HAL_UART_ERROR_ORE);
    dma.State = HAL_DMA_STATE_ABORT;

    service_recovery();
    assert(recovery.state == UART_RX_RECOVERY_WAIT_DMA);
    assert(hal_mock_uart_abort_count() == 0U);
    assert(hal_mock_uart_start_count() == 0U);
    assert(uart_rx_recovery_commands_allowed(&recovery) == 0U);

    dma.State = HAL_DMA_STATE_READY;
    service_recovery();
    assert(recovery.state == UART_RX_RECOVERY_ACTIVE);
    assert(hal_mock_uart_abort_count() == 1U);
    assert(hal_mock_uart_start_count() == 1U);
}

static void test_abort_failure_retries_after_backoff(void)
{
    reset_fixture();
    hal_mock_uart_set_abort_result(HAL_ERROR);
    report_error(HAL_UART_ERROR_DMA);

    service_recovery();
    assert(recovery.state == UART_RX_RECOVERY_BACKOFF);
    assert(recovery.last_failure == UART_RX_FAILURE_ABORT);
    assert(hal_mock_uart_abort_count() == 1U);
    assert(hal_mock_uart_start_count() == 0U);

    hal_mock_uart_set_abort_result(HAL_OK);
    hal_mock_advance_tick(UART_RX_RECOVERY_RETRY_INITIAL_MS);
    service_recovery();

    assert(recovery.state == UART_RX_RECOVERY_ACTIVE);
    assert(hal_mock_uart_abort_count() == 2U);
    assert(hal_mock_uart_start_count() == 1U);
}

static void test_start_busy_is_retried_and_limited(void)
{
    reset_fixture();
    hal_mock_uart_set_start_result(HAL_BUSY);
    report_error(HAL_UART_ERROR_FE);

    service_recovery();
    assert(recovery.state == UART_RX_RECOVERY_BACKOFF);
    assert(recovery.last_failure == UART_RX_FAILURE_START);

    for (uint8_t attempt = 1U;
         attempt < UART_RX_RECOVERY_MAX_ATTEMPTS;
         attempt++)
    {
        hal_mock_advance_tick(UART_RX_RECOVERY_RETRY_MAX_MS);
        service_recovery();
        assert(recovery.state == UART_RX_RECOVERY_BACKOFF ||
               recovery.state == UART_RX_RECOVERY_FAULT);
    }

    assert(recovery.state == UART_RX_RECOVERY_FAULT);
    assert(recovery.last_failure == UART_RX_FAILURE_RETRY_LIMIT);
    assert(hal_mock_uart_start_count() ==
           UART_RX_RECOVERY_MAX_ATTEMPTS);

    hal_mock_advance_tick(UART_RX_RECOVERY_RETRY_MAX_MS * 4U);
    service_recovery();
    assert(hal_mock_uart_start_count() ==
           UART_RX_RECOVERY_MAX_ATTEMPTS);
}

static void test_dma_not_ready_timeout_latches_fault(void)
{
    reset_fixture();
    report_error(HAL_UART_ERROR_ORE);
    dma.State = HAL_DMA_STATE_ABORT;

    service_recovery();
    hal_mock_advance_tick(UART_RX_RECOVERY_DMA_WAIT_TIMEOUT_MS);
    service_recovery();

    assert(recovery.state == UART_RX_RECOVERY_FAULT);
    assert(recovery.last_failure == UART_RX_FAILURE_DMA_NOT_READY);
    assert(hal_mock_uart_abort_count() == 0U);
    assert(hal_mock_uart_start_count() == 0U);
}

static void test_recoverable_notification_does_not_restart_active_rx(void)
{
    reset_fixture();
    uart.RxState = HAL_UART_STATE_BUSY_RX;
    uart.ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    dma.State = HAL_DMA_STATE_BUSY;
    report_error(HAL_UART_ERROR_NE);

    service_recovery();

    assert(recovery.state == UART_RX_RECOVERY_ACTIVE);
    assert(hal_mock_uart_abort_count() == 0U);
    assert(hal_mock_uart_start_count() == 0U);
}

static void test_restart_drops_partial_then_accepts_immediate_stop(void)
{
    const char partial[] = "SEQ:42 CMD:HAND_STOP";
    const char next_frame[] = "SEQ:43 CMD:HAND_STOP\n";
    char line[UART_LINE_BUFFER_SIZE];
    hand_command_t command;

    reset_fixture();
    for (size_t i = 0U; partial[i] != '\0'; i++)
    {
        dma_buffer[i] = (uint8_t)partial[i];
    }
    uart_dma_rx_collect(&dma_rx, dma_buffer,
                        (uint16_t)strlen(partial), &line_rx);
    assert(uart_line_rx_poll(&line_rx, line) == 0U);
    assert(line_rx.line_index != 0U);

    /* HAL abort doesn't drain DR; recovery must clear it before restart. */
    usart.SR = 0x20U;
    usart.DR = (uint32_t)'X';
    report_error(HAL_UART_ERROR_ORE);
    service_recovery();

    assert(recovery.state == UART_RX_RECOVERY_ACTIVE);
    assert(dma_rx.last_pos == 0U);
    assert(line_rx.line_index == 0U);
    assert(line_rx.line_discard == 0U);
    assert(uart_line_rx_pending_for_test(&line_rx) == 0U);
    assert(usart.SR == 0U);
    assert(usart.DR == 0U);
    assert(hal_mock_uart_clear_overrun_count() == 1U);

    memcpy(dma_buffer, next_frame, strlen(next_frame));
    uart_dma_rx_collect(&dma_rx, dma_buffer,
                        (uint16_t)strlen(next_frame), &line_rx);

    assert(uart_line_rx_poll(&line_rx, line) == 1U);
    assert(hand_protocol_parse_frame(line, &command));
    assert(command.seq == 43U);
    assert(command.action == HAND_ACTION_STOP);
}

static void restart_receiver_cleanly(void)
{
    report_error(HAL_UART_ERROR_ORE);
    service_recovery();
    assert(recovery.state == UART_RX_RECOVERY_ACTIVE);
    assert(dma_rx.last_pos == 0U);
    assert(line_rx.line_index == 0U);
    assert(line_rx.line_discard == 0U);
    assert(uart_line_rx_pending_for_test(&line_rx) == 0U);
}

static void test_invalid_prefix_after_restart_cannot_form_command(void)
{
    const char invalid_then_stop[] =
        "?SEQ:51 CMD:HAND_STOP\nSEQ:52 CMD:HAND_STOP\n";
    char line[UART_LINE_BUFFER_SIZE];
    hand_command_t command;

    reset_fixture();
    restart_receiver_cleanly();

    memcpy(dma_buffer, invalid_then_stop, strlen(invalid_then_stop));
    uart_dma_rx_collect(&dma_rx, dma_buffer,
                        (uint16_t)strlen(invalid_then_stop), &line_rx);
    assert(uart_line_rx_poll(&line_rx, line) == 1U);
    assert(!hand_protocol_parse_frame(line, &command));

    assert(uart_line_rx_poll(&line_rx, line) == 1U);
    assert(hand_protocol_parse_frame(line, &command));
    assert(command.seq == 52U);
    assert(command.action == HAND_ACTION_STOP);
}

static void test_two_complete_frames_after_restart_are_separate(void)
{
    const char frames[] =
        "SEQ:61 CMD:HAND_STOP\nSEQ:62 CMD:HAND_OPEN\n";
    char line[UART_LINE_BUFFER_SIZE];
    hand_command_t command;

    reset_fixture();
    restart_receiver_cleanly();

    memcpy(dma_buffer, frames, strlen(frames));
    uart_dma_rx_collect(&dma_rx, dma_buffer,
                        (uint16_t)strlen(frames), &line_rx);

    assert(uart_line_rx_poll(&line_rx, line) == 1U);
    assert(hand_protocol_parse_frame(line, &command));
    assert(command.seq == 61U);
    assert(command.action == HAND_ACTION_STOP);

    assert(uart_line_rx_poll(&line_rx, line) == 1U);
    assert(hand_protocol_parse_frame(line, &command));
    assert(command.seq == 62U);
    assert(command.action == HAND_ACTION_OPEN);
}

static void test_software_overflow_still_discards_until_newline(void)
{
    const char stop_frame[] = "SEQ:71 CMD:HAND_STOP\n";
    const char next_frame[] = "SEQ:72 CMD:HAND_STOP\n";
    char line[UART_LINE_BUFFER_SIZE];
    hand_command_t command;

    reset_fixture();
    for (uint16_t i = 0U; i < UART_LINE_RING_SIZE; i++)
    {
        uart_line_rx_push_isr(&line_rx, (uint8_t)'X');
    }
    assert(uart_line_rx_has_overflow(&line_rx) == 1U);
    uart_line_rx_recover_overflow(&line_rx);
    assert(uart_line_rx_has_overflow(&line_rx) == 0U);
    assert(line_rx.line_discard == 1U);
    assert(uart_line_rx_pending_for_test(&line_rx) == 0U);

    for (size_t i = 0U; i < strlen(stop_frame); i++)
    {
        uart_line_rx_push_isr(&line_rx, (uint8_t)stop_frame[i]);
    }
    assert(uart_line_rx_poll(&line_rx, line) == 0U);

    for (size_t i = 0U; i < strlen(next_frame); i++)
    {
        uart_line_rx_push_isr(&line_rx, (uint8_t)next_frame[i]);
    }
    assert(uart_line_rx_poll(&line_rx, line) == 1U);
    assert(hand_protocol_parse_frame(line, &command));
    assert(command.seq == 72U);
    assert(command.action == HAND_ACTION_STOP);
}

static void test_critical_section_preserves_mask_state(void)
{
    reset_fixture();
    hal_mock_set_primask(1U);
    report_error(HAL_UART_ERROR_NE);
    assert(hal_mock_get_primask() == 1U);
    service_recovery();
    assert(hal_mock_get_primask() == 1U);
    hal_mock_set_primask(0U);
}

int main(void)
{
    test_ore_is_recorded_without_hal_work_in_callback();
    test_fe_ne_and_dma_error_counts();
    test_non_usart1_error_is_ignored();
    test_dma_abort_in_progress_blocks_restart();
    test_abort_failure_retries_after_backoff();
    test_start_busy_is_retried_and_limited();
    test_dma_not_ready_timeout_latches_fault();
    test_recoverable_notification_does_not_restart_active_rx();
    test_restart_drops_partial_then_accepts_immediate_stop();
    test_invalid_prefix_after_restart_cannot_form_command();
    test_two_complete_frames_after_restart_are_separate();
    test_software_overflow_still_discards_until_newline();
    test_critical_section_preserves_mask_state();
    return 0;
}
