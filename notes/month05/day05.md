# 第五个月 Day5：完善具身智能灵动手 README

## 今日目标

完善具身智能灵动手执行端文档，重点整理 PCA9685、舵机 PWM、动作库、状态机、安全限幅、异常保护和供电注意事项。

## 今日完成内容

1. 检查灵动手执行端真实代码
2. 区分 embodied-robotic-hand 和 stm32_f407_hand_controller 的仓库职责
3. 整理 PCA9685 控制流程
4. 整理五路舵机通道
5. 整理 HAND_OPEN、HAND_GRAB、HAND_RELEASE、HAND_STOP 动作
6. 整理动作状态切换
7. 整理舵机角度限幅
8. 整理非法命令处理
9. 整理独立供电和共地要求
10. 补充实验现象和面试可讲点

## 系统执行链路

```text
Linux Gateway 或上位机
        ↓ UART
STM32F407 控制器
        ↓ 协议解析
动作命令分发
        ↓
舵机角度限幅
        ↓ I2C
PCA9685
        ↓ PWM
五路 MG90S 舵机
        ↓
灵动手执行动作