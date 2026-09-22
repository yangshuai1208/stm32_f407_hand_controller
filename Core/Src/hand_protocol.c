#include "hand_protocol.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
bool hand_protocol_parse_frame(const char*line, hand_command_t*command)
{
    if(line==NULL||command==NULL)
    {
        return false;
    }
    if(strncmp(line,"SEQ:",4)!=0)
    {
        return false;           
    }
    char *end_ptr=NULL;

    unsigned long seq=strtoul(line+4,&end_ptr,10);

    if(end_ptr==line+4)
    {
        return false;
    }
    if(strncmp(end_ptr," CMD:",5)!=0)
    {
        return false;
    }
    const char *cmd_text=end_ptr+5;

    hand_action_t action=hand_protocol_parse(cmd_text);

    if(action==HAND_ACTION_NONE)
    {
        return false;
    }
    command->seq=(uint32_t)seq;
    command->action=action;
    
    return true;
}
