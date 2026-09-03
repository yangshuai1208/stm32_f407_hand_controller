#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HAND_COMMAND_MAX_LENGTH 16U

typedef enum
{
    HAND_COMMAND_UNKNOWN = 0,
    HAND_COMMAND_OPEN,
    HAND_COMMAND_GRAB,
    HAND_COMMAND_RELEASE,
    HAND_COMMAND_STOP
} HandCommand;

static bool BytesEqual(const uint8_t *data,
                       size_t data_length,
                       const char *expected)
{
    /* TODO 1：
     * 1. 获取 expected 的长度
     * 2. 先比较长度
     * 3. 再使用 memcmp 比较内容
     */
    size_t expected_length=strlen(expected);
    if(data_length!=expected_length)
    {
        return false;
    }
    if(memcmp(data,expected,expected_length)!=0)
    {
        return false;
    }
    return true;
}

bool HandCommand_Parse(const uint8_t *data,
                       size_t length,
                       HandCommand *command_out)
{
    HandCommand parsed_command;

    if ((data == NULL) ||
        (command_out == NULL) ||
        (length == 0U) ||
        (length > HAND_COMMAND_MAX_LENGTH))
    {
        return false;
    }

    while ((length > 0U) &&
           ((data[length - 1U] == '\r') ||
            (data[length - 1U] == '\n')))
    {
        length--;
    }

    if (length == 0U)
    {
        return false;
    }

    if (BytesEqual(data, length, "HAND_OPEN"))
    {
        parsed_command = HAND_COMMAND_OPEN;
    }
    else if (BytesEqual(data, length, "HAND_GRAB"))
    {
        parsed_command = HAND_COMMAND_GRAB;
    }
    else if (BytesEqual(data, length, "HAND_RELEASE"))
    {
        parsed_command = HAND_COMMAND_RELEASE;
    }
    else if (BytesEqual(data, length, "HAND_STOP"))
    {
        parsed_command = HAND_COMMAND_STOP;
    }
    else
    {
        return false;
    }

    *command_out = parsed_command;
    return true;
}

static void TestValidCommands(void)
{
    static const uint8_t open_data[] = "HAND_OPEN\r\n";
    static const uint8_t grab_data[] = "HAND_GRAB\n";
    static const uint8_t release_data[] = "HAND_RELEASE";
    static const uint8_t stop_data[] = "HAND_STOP\r\n";
  

    HandCommand command = HAND_COMMAND_UNKNOWN;





    assert(HandCommand_Parse(open_data,
                             sizeof(open_data) - 1U,
                             &command));
    assert(command == HAND_COMMAND_OPEN);

    assert(HandCommand_Parse(grab_data,
                             sizeof(grab_data) - 1U,
                             &command));
    assert(command == HAND_COMMAND_GRAB);

    assert(HandCommand_Parse(release_data,
                             sizeof(release_data) - 1U,
                             &command));
    assert(command == HAND_COMMAND_RELEASE);

    assert(HandCommand_Parse(stop_data,
                             sizeof(stop_data) - 1U,
                             &command));
    assert(command == HAND_COMMAND_STOP);
}

static void TestInvalidCommands(void)
{
    static const uint8_t extra_data[] = "HAND_OPEN_ERROR";
    static const uint8_t incomplete_data[] = "HAND_OP";

    HandCommand command = HAND_COMMAND_STOP;

    assert(!HandCommand_Parse(extra_data,
                              sizeof(extra_data) - 1U,
                              &command));
    assert(command == HAND_COMMAND_STOP);

    assert(!HandCommand_Parse(incomplete_data,
                              sizeof(incomplete_data) - 1U,
                              &command));
    assert(command == HAND_COMMAND_STOP);

    assert(!HandCommand_Parse(NULL, 5U, &command));
    assert(!HandCommand_Parse(incomplete_data, 0U, &command));
    assert(!HandCommand_Parse(incomplete_data,
                              sizeof(incomplete_data) - 1U,
                              NULL));
}

int main(void)
{
    TestValidCommands();
    TestInvalidCommands();

    printf("Day15 hand command parser tests passed\n");
    return 0;
}