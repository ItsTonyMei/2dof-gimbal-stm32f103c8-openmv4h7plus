# CLAUDE.md — 2-DOF Gimbal STM32F103C8 + OpenMV4 H7 Plus / N6

## Project Overview

Two-degree-of-freedom servo gimbal for person tracking. Two hardware architectures coexist in this repo:

| Architecture | Vision | Control | Status |
|---|---|---|---|
| N6 Single-Board | OpenMV N6 (BlazeFace NPU) | PD on N6, direct PWM | Current |
| H7+ + STM32 | OpenMV4 H7 Plus (color blob) | PD on STM32F103C8T6 via UART3 | Legacy |

## Build System

- **STM32**: Keil MDK (Arm Compiler 6) via `stm32/MDK-ARM/MiniBalance.uvprojx`, or GCC via `build_gcc.sh`
- **OpenMV**: MicroPython, copy `.py` to camera flash
- **Flashing**: `tools/FlyMcu/FlyMcu.exe` (ISP via CH9102 USB-UART)

## Architecture

### Current (N6 Single-Board)
- `openmv/main.py`: BlazeFace detection → PD controller → PWM on P7(yaw) P9(pitch)
- No external MCU needed — everything runs on OpenMV N6
- Triple deadzone: position (inner 13px, outer 27px), derivative (5px), PWM output (5us)
- Lost-target timeout with auto return-to-center

### Legacy (H7+ + STM32)
- `openmv/color-tracking.py`: LAB color blob detection → UART3 TX
- `stm32/`: Receives 5-byte protocol `[0xFF][0xFE][hasBlob][tx][ty]` → PD → TIM4 PWM
- STM32F103C8T6: 72MHz, 64KB Flash, 20KB RAM

## Available Agents & Skills

- `cortex-debugger`: STM32/ESP32 firmware crash analysis (HardFault, stack overflow, DMA, interrupts)
- `protocol-analyzer`: UART/serial protocol debugging (5-byte frame, MAVLink, custom protocols)

## Key Files

- `openmv/main.py` — current N6 firmware
- `stm32/Core/Src/main.c` — STM32 main loop (legacy)
- `stm32/MiniBalance/CONTROL/control.c` — PD controller (legacy)
- `stm32/Core/Src/usart.c` — 5-byte protocol parser (legacy)
