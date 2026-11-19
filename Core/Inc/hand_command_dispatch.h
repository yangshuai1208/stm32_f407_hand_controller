#ifndef HAND_COMMAND_DISPATCH_H
#define HAND_COMMAND_DISPATCH_H

#include <stdint.h>

#include "hand_ack_cache.h"
#include "hand_comm_safety.h"
#include "hand_protocol.h"

typedef uint8_t (*hand_command_is_busy_fn)(void);
typedef HAL_StatusTypeDef (*hand_command_update_fn)(void);
typedef void (*hand_command_send_ack_fn)(uint32_t seq,
                                         ack_status_t status);
typedef void (*hand_command_send_seq_conflict_fn)(void);

typedef struct
{
    hand_comm_start_action_fn start_action;
    hand_command_is_busy_fn is_busy;
    hand_command_send_ack_fn send_ack;
    hand_command_send_seq_conflict_fn send_seq_conflict;
} hand_command_dispatch_ops_t;

void hand_command_dispatch(
    const hand_command_t *command,
    uint8_t *action_in_progress,
    uint32_t *active_seq,
    hand_action_t *active_action,
    hand_ack_cache_t *ack_cache,
    hand_comm_safety_t *comm_safety,
    const hand_command_dispatch_ops_t *ops);

/* Advance the servo once, then settle the active command's ACK. */
void hand_command_service_motion(
    uint8_t *action_in_progress,
    uint32_t *active_seq,
    hand_action_t *active_action,
    hand_ack_cache_t *ack_cache,
    hand_command_update_fn update,
    hand_command_is_busy_fn is_busy,
    hand_command_send_ack_fn send_ack);

#endif
