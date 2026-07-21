#include "hand_protocol.h"

#include <string.h>
#include <stdio.h>

hand_action_t hand_protocol_parse(const char *cmd)
{
    if(cmd==NULL)
    {
        return HAND_ACTION_NONE;
    }
    if (strcmp(cmd,"HAND_OPEN")==0)
    {
        return HAND_ACTION_OPEN;
    }
    if(strcmp(cmd,"HAND_GRAB")==0)
    {
        return HAND_ACTION_GRAB;
    }
    if(strcmp(cmd,"HAND_RELEASE")==0)
    {
        return HAND_ACTION_RELEASE;
    }
    if(strcmp(cmd,"HAND_STOP")==0)
    {
        return HAND_ACTION_STOP;
    }
    return HAND_ACTION_NONE;
}
const char *hand_action_to_string(hand_action_t action)
{
    switch (action)
    {
    case HAND_ACTION_OPEN:
        return "HAND_ACTION_OPEN";
    case HAND_ACTION_GRAB:
          return "HAND_ACTION_GRAB";
    case HAND_ACTION_RELEASE:
          return "HAND_ACTION_RELEASE";
    case HAND_ACTION_STOP:
          return "HAND_ACTION_STOP";
    case HAND_ACTION_NONE:
    default:
        return "HAND_ACTION_NONE";
    }
}
void hand_action_execute(hand_action_t action)
{
switch (action)
    {
    case HAND_ACTION_OPEN:
          printf("ACTION: open hand\r\n");
          break;
    case HAND_ACTION_GRAB:
           printf("ACTION: grab object\r\n");
           break;

    case HAND_ACTION_RELEASE:
           printf("ACTION: release object\r\n");
           break;

    case HAND_ACTION_STOP:
          printf("ACTION: emergency stop\r\n");
          break;

    case HAND_ACTION_NONE:
    default:
       printf("ACTION: no valid command\r\n");
        break;
    }
}
