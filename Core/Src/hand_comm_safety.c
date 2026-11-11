#include "hand_comm_safety.h"

void hand_comm_safety_init(hand_comm_safety_t *safety)
{
    if (safety == 0)
    {
        return;
    }

    safety->uart_error_pending = 0U;
    safety->stop_attempted = 0U;
    safety->attempted_seq = 0U;
    safety->attempted_action = HAND_ACTION_NONE;
}

void hand_comm_safety_notify_uart_error_isr(hand_comm_safety_t *safety)
{
    if (safety != 0)
    {
        safety->uart_error_pending = 1U;
    }
}

void hand_comm_safety_on_action_started(hand_comm_safety_t *safety)
{
    if (safety != 0)
    {
        safety->stop_attempted = 0U;
        safety->attempted_seq = 0U;
        safety->attempted_action = HAND_ACTION_NONE;
    }
}

static uint8_t hand_comm_safety_take_uart_error(hand_comm_safety_t *safety)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t pending = safety->uart_error_pending;
    safety->uart_error_pending = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return pending;
}

hand_comm_safety_result_t hand_comm_safety_service(
    hand_comm_safety_t *safety,
    uint8_t *action_in_progress,
    uint32_t *active_seq,
    hand_action_t *active_action,
    hand_ack_cache_t *ack_cache,
    hand_comm_start_action_fn start_action)
{
    if (safety == 0 || action_in_progress == 0 || active_seq == 0 ||
        active_action == 0 || ack_cache == 0 || start_action == 0)
    {
        return HAND_COMM_SAFETY_NO_EVENT;
    }

    if (hand_comm_safety_take_uart_error(safety) == 0U)
    {
        return HAND_COMM_SAFETY_NO_EVENT;
    }

    if (*action_in_progress == 0U)
    {
        return HAND_COMM_SAFETY_NO_ACTIVE_ACTION;
    }

    if (safety->stop_attempted != 0U &&
        safety->attempted_seq == *active_seq &&
        safety->attempted_action == *active_action)
    {
        return HAND_COMM_SAFETY_ALREADY_ATTEMPTED;
    }

    safety->stop_attempted = 1U;
    safety->attempted_seq = *active_seq;
    safety->attempted_action = *active_action;

    if (start_action(HAND_ACTION_STOP) != HAL_OK)
    {
        return HAND_COMM_SAFETY_STOP_FAILED;
    }

    hand_ack_record_t *record =
        hand_ack_cache_find(ack_cache, *active_seq);
    if (record == 0 || record->status != ACK_STATUS_ERROR)
    {
        hand_ack_cache_set(ack_cache,
                           *active_seq,
                           *active_action,
                           ACK_STATUS_PREEMPTED);
    }

    *action_in_progress = 0U;
    *active_seq = 0U;
    *active_action = HAND_ACTION_NONE;
    safety->stop_attempted = 0U;
    safety->attempted_seq = 0U;
    safety->attempted_action = HAND_ACTION_NONE;

    return HAND_COMM_SAFETY_ACTION_CANCELLED;
}
