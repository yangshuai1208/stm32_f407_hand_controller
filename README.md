第六阶段 Day15：UART命令安全解析与可靠通信复习

新增主机侧单元测试：

test/day15_hand_command_parser.c

本次完成：

使用“原始字节指针＋实际长度”解析UART文本命令。

支持HAND_OPEN、HAND_GRAB、HAND_RELEASE和HAND_STOP。

支持清理命令末尾的\r和\n。

使用长度比较与memcmp()进行精确匹配。

拒绝不完整命令、额外后缀、空输入、纯换行和超长输入。

参数校验失败或命令非法时不修改调用者的输出参数。

复习UART字节流组帧、sequence、ACK、超时重传和重复请求去重。

在Windows PowerShell中通过WSL编译：

wsl gcc -std=c11 -Wall -Wextra -Wpedantic -Werror ./test/day15_hand_command_parser.c -o ./test/day15_hand_command_parser
wsl ./test/day15_hand_command_parser

验证结果：

Day15 hand command parser tests passed

实现边界：本次新增内容是可在Linux主机环境运行的UART命令解析单元测试。sequence、ACK等待、超时重传和去重主要在独立C++实验中验证，尚未全部接入真实UART与STM32执行反馈链路，不能将其描述为已完成硬件端到端可靠闭环。