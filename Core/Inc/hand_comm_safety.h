#ifndef HAND_COMM_SAFETY_H
#define HAND_COMM_SAFETY_H

#include <stdint.h>

#include "hand_ack_cache.h"
#include "hand_servo.h"

typedef HAL_StatusTypeDef (*hand_comm_start_action_fn)(hand_action_t action);

typedef enum
{
    HAND_COMM_SAFETY_NO_EVENT = 0U,
    HAND_COMM_SAFETY_NO_ACTIVE_ACTION,
    HAND_COMM_SAFETY_ACTION_CANCELLED,
    HAND_COMM_SAFETY_STOP_FAILED,
    HAND_COMM_SAFETY_ALREADY_ATTEMPTED
} hand_comm_safety_result_t;

typedef struct
{
    volatile uint8_t uart_error_pending;
    uint8_t stop_attempted;
    uint32_t attempted_seq;
    hand_action_t attempted_action;
} hand_comm_safety_t;

void hand_comm_safety_init(hand_comm_safety_t *safety);

/* Safe to call from the UART error ISR; this only latches an event. */
void hand_comm_safety_notify_uart_error_isr(hand_comm_safety_t *safety);

/* Reset the one-attempt guard when the main loop accepts a new action. */
void hand_comm_safety_on_action_started(hand_comm_safety_t *safety);

/* Call from the main loop before command dispatch and servo update. */
hand_comm_safety_result_t hand_comm_safety_service(
    hand_comm_safety_t *safety,
    uint8_t *action_in_progress,
    uint32_t *active_seq,
    hand_action_t *active_action,
    hand_ack_cache_t *ack_cache,
    hand_comm_start_action_fn start_action);

#endif
