# 2-DOF 云台项目文档

## 项目概述

基于 STM32F103C8T6 + OpenMV4 H7 Plus 的二自由度颜色追踪云台。

- **MCU**: STM32F103C8T6
- **视觉**: OpenMV4 H7 Plus（颜色追踪，30-60fps）
- **通信**: UART3 (115200 8N1)，5字节协议

## 目录

1. [硬件与接线](./01-硬件与接线/hardware.md) - 硬件说明、原理图解读
2. [开发环境](./02-开发环境/dev-environment.md) - 开发工具、驱动安装
3. [快速开始](./03-快速开始/quick-start.md) - 从开箱到运行的快速指南
4. [工程源码解析](./04-工程源码解析/source-code.md) - 源码结构
5. [通讯协议](./05-通讯协议/protocol.md) - 串口指令、OpenMV协议
6. [ROS与Python例程](./06-ROS与Python例程/ros-python.md) - 上层控制例程
7. [开发笔记](./07-开发笔记/dev-notes.md) - 原始开发记录整理
8. [C06B 360°舵机驱动例程](./08-C06B-360舵机驱动例程/360-servo-c06b.md) - C06B 板 360° 舵机独立例程分析
9. [OpenMV颜色追踪](./09-OpenMV人形追踪/openmv-tracking.md) - OpenMV4 H7 Plus 颜色追踪模式

## OpenMV 颜色追踪（新增）

本项目在 WHEELTEC 云台基础上新增 OpenMV4 H7 Plus 颜色追踪功能，Mode 2 模式。
详见 [OpenMV颜色追踪](./09-OpenMV人形追踪/openmv-tracking.md)。

## 相关文档

- [软死区调参日志](./work-log-soft-deadzone.md) - 2026-05-02 调参完整记录
