# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.2.0] - 2026-06-16

### Added
- STM32F103RCT6 完整固件 (`stm32_firmware/`)，PlatformIO + Arduino 框架
  - SBUS 遥控器接收 (USART2 PA3, 100000bps 8E2, 三极管反相)
  - X5 MotorCmd 协议解析 (USART1 PA9/PA10, CH340N Micro USB)
  - 坦克混控 + 双路 Servo PWM (S1=PC3 左电调, S2=PC2 右电调)
  - 控制优先级仲裁: SBUS ARM > X5 自主 > 60s 超时刹停
  - 蜂鸣器提示 (PC5) + LED 状态指示 (PB5)
  - MPU9250 IMU SPI 基础读取 (PB12-15)
- V3.0 扩展板引脚定义与原理图交叉验证

### Lessons Learned (踩坑记录)
- `genericSTM32F103RC` 变体默认不映射 `Serial` 到 USART1，需显式 `Serial.setRx(PA10)` / `Serial.setTx(PA9)`
- PC0-PC3 在 LQFP64 封装无硬件 TIM 通道，必须用 Arduino Servo 库（不能用原 C06B 的寄存器操作法）
- V3.0 自动下载电路不兼容 C06B 的 DTR/RTS 时序，暂需手动 BOOT0+RESET 进 bootloader
- MPU9250 在此板上走 SPI（非 I2C），WHO_AM_I=0x71 确认
- ZTW Seal G2 电调中位 1275μs (非标)、PWM 范围 650-1900μs 保持不变
- CRC8 poly 0x07 与 ESP32/ESP8266/X5 三方一致
- 编译资源: RAM 4.0% (1964/49152), Flash 8.4% (21980/262144)

## [0.1.0] - 2026-06-15

### Added
- Project skeleton: directory structure, README, LICENSE, .gitignore
- Architecture design: dual-layer (RDK X5 L2 + STM32 L1) with SBUS safety override
- Documentation placeholders: architecture, hardware setup, protocol spec, migration plan, safety design
- ROS2 package scaffold: `src/tracked_vehicle/` with module stubs
