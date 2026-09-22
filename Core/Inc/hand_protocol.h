#ifndef HAND_PROTOCOL_H
#define HAND_PROTOCOL_H

typedef enum
{
    HAND_ACTION_NONE=0,
    HAND_ACTION_OPEN,
    HAND_ACTION_GRAB,
    HAND_ACTION_RELEASE,
    HAND_ACTION_STOP
}hand_action_t;


#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t seq;
    hand_action_t action;
}hand_command_t;

hand_action_t hand_protocol_parse(const char *cmd);
const char *hand_action_to_string(hand_action_t action);
void hand_action_execute(hand_action_t action);
bool hand_protocol_parse_frame(const char*line, hand_command_t*command);




#endif
