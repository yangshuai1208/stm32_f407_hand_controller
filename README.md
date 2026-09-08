# STM32F407 五指灵动手执行端

## 项目简介

本项目是 AIoT 智能眼镜与灵动手控制系统中的 STM32 执行端，基于 STM32F407VET6、PCA9685 和 5 路 MG90S 舵机完成五指动作控制。

STM32 通过 USART1 接收上位机发送的文本命令，解析 `HAND_OPEN`、`HAND_GRAB`、`HAND_RELEASE`、`HAND_STOP`，再通过 I2C1 配置 PCA9685 输出 5 路 PWM，驱动五个手指舵机运动。动作执行后通过串口返回业务执行 ACK。

```text
Linux / 上位机
    ↓ USART1 115200 8N1
STM32F407
    ↓ 命令解析
HAND_OPEN / HAND_GRAB / HAND_RELEASE / HAND_STOP
    ↓
I2C1
    ↓
PCA9685
    ↓ 5路PWM
MG90S × 5