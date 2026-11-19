#include "hand_command_dispatch.h"

void hand_command_dispatch(
    const hand_command_t *command,
    uint8_t *action_in_progress,
    uint32_t *active_seq,
    hand_action_t *active_action,
    hand_ack_cache_t *ack_cache,
    hand_comm_safety_t *comm_safety,
    const hand_command_dispatch_ops_t *ops)
{
    if (command == 0 || action_in_progress == 0 || active_seq == 0 ||
        active_action == 0 || ack_cache == 0 || comm_safety == 0 ||
        ops == 0 || ops->start_action == 0 || ops->is_busy == 0 ||
        ops->send_ack == 0 || ops->send_seq_conflict == 0)
    {
        return;
    }

    /* Check the active sequence before consulting the completed ACK cache. */
    uint8_t active_seq_handled = 0U;

    if (*action_in_progress != 0U && command->seq == *active_seq)
    {
        if (command->action != *active_action)
        {
            ops->send_seq_conflict();
        }
        else
        {
            ops->send_ack(command->seq, ACK_STATUS_IN_PROGRESS);
        }

        active_seq_handled = 1U;
    }

    if (active_seq_handled != 0U)
    {
        return;
    }

    hand_ack_record_t *old_record =
        hand_ack_cache_find(ack_cache, command->seq);

    if (old_record != 0)
    {
        if (old_record->action != command->action)
        {
            ops->send_seq_conflict();
        }
        else
        {
            ops->send_ack(command->seq, old_record->status);
        }

        return;
    }

    /* STOP has priority over an active ordinary action. */
    if (command->action == HAND_ACTION_STOP)
    {
        uint8_t had_active = *action_in_progress;
        uint32_t old_seq = *active_seq;
        hand_action_t old_action = *active_action;

        if (ops->start_action(HAND_ACTION_STOP) == HAL_OK)
        {
            hand_ack_cache_set(ack_cache,
                               command->seq,
                               command->action,
                               ACK_STATUS_OK);

            /* Preserve the original ACK order: old PREEMPTED, then STOP OK. */
            if (had_active != 0U)
            {
                hand_ack_cache_set(ack_cache,
                                   old_seq,
                                   old_action,
                                   ACK_STATUS_PREEMPTED);
                ops->send_ack(old_seq, ACK_STATUS_PREEMPTED);
            }

            *active_seq = 0U;
            *active_action = HAND_ACTION_NONE;
            *action_in_progress = 0U;

            ops->send_ack(command->seq, ACK_STATUS_OK);
        }
        else
        {
            hand_ack_cache_set(ack_cache,
                               command->seq,
                               command->action,
                               ACK_STATUS_ERROR);
            ops->send_ack(command->seq, ACK_STATUS_ERROR);
        }

        return;
    }

    /* Ordinary actions may start only while the servo state machine is idle. */
    if (ops->is_busy() == 0U)
    {
        if (ops->start_action(command->action) == HAL_OK)
        {
            hand_comm_safety_on_action_started(comm_safety);
            *active_seq = command->seq;
            *active_action = command->action;
            *action_in_progress = 1U;

            hand_ack_cache_set(ack_cache,
                               command->seq,
                               command->action,
                               ACK_STATUS_IN_PROGRESS);
            ops->send_ack(command->seq, ACK_STATUS_IN_PROGRESS);
        }
        else
        {
            hand_ack_cache_set(ack_cache,
                               command->seq,
                               command->action,
                               ACK_STATUS_ERROR);
            ops->send_ack(command->seq, ACK_STATUS_ERROR);
        }
    }
    else
    {
        hand_ack_cache_set(ack_cache,
                           command->seq,
                           command->action,
                           ACK_STATUS_BUSY);
        ops->send_ack(command->seq, ACK_STATUS_BUSY);
    }
}

void hand_command_service_motion(
    uint8_t *action_in_progress,
    uint32_t *active_seq,
    hand_action_t *active_action,
    hand_ack_cache_t *ack_cache,
    hand_command_update_fn update,
    hand_command_is_busy_fn is_busy,
    hand_command_send_ack_fn send_ack)
{
    if (action_in_progress == 0 || active_seq == 0 ||
        active_action == 0 || ack_cache == 0 || update == 0 ||
        is_busy == 0 || send_ack == 0)
    {
        return;
    }

    HAL_StatusTypeDef update_ret = update();

    if (update_ret != HAL_OK)
    {
        if (*action_in_progress != 0U)
        {
            hand_ack_cache_set(ack_cache,
                               *active_seq,
                               *active_action,
                               ACK_STATUS_ERROR);
            send_ack(*active_seq, ACK_STATUS_ERROR);
        }

        *action_in_progress = 0U;
        *active_action = HAND_ACTION_NONE;
        *active_seq = 0U;
    }

    if (*action_in_progress != 0U && is_busy() == 0U)
    {
        hand_ack_cache_set(ack_cache,
                           *active_seq,
                           *active_action,
                           ACK_STATUS_OK);
        send_ack(*active_seq, ACK_STATUS_OK);

        *action_in_progress = 0U;
        *active_action = HAND_ACTION_NONE;
        *active_seq = 0U;
    }
}
