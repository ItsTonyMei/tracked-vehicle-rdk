# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.2.1] - 2026-06-19

### Fixed
- PWM 参数改为标准值: 中位 1500μs, 范围 1000-2000μs
- ZTW Seal G2 此批次需标准舵机 PWM, 1275μs 非标值导致 ESC 无法完成自检

## [0.2.0] - 2026-06-17

### Added
- STM32F103RCT6 完整固件 (`stm32_firmware/`)，PlatformIO + Arduino 框架
  - SBUS 遥控器接收 (USART2 PA3, 100000bps 8E2, 三极管反相, WFLY RF209S 适配)
  - X5 MotorCmd 协议解析 (USART1 PA9/PA10, CH340N Micro USB, 6字节 CRC8)
  - 坦克混控 + 双路 Servo PWM (S1=PC3 左电调, S2=PC2 右电调, 50Hz)
  - 控制优先级: SBUS ARM > X5 自主 > 60s 超时刹停
  - CH5 ARM/DISARM 防抖 (3帧确认) + 信号丢失需手动重新 ARM
  - CH6 手控/自动模式切换 (LOW=手控, HIGH=自动) + 非阻塞蜂鸣
  - SBUS 信号防抖: 5帧确认有效, 2次连续超时判丢失, 悬空引脚不误判
  - 蜂鸣器提示 (PC5) + LED 状态指示 (PC13, 快/中/慢三速闪烁)
  - MPU9250 IMU SPI 基础读取 (PB12-15, WHO_AM_I=0x71)
  - 5Hz 串口状态输出 (含 CH5/CH6/IMU 角速度)
- V3.0 扩展板引脚定义与原理图交叉验证
- 完整硬件引脚表 + 通信协议文档 (`docs/`)

### Lessons Learned (踩坑记录)
- `genericSTM32F103RC` 变体默认不映射 `Serial` 到 USART1，需显式 `Serial.setRx(PA10)` / `Serial.setTx(PA9)`
- PC0-PC3 在 LQFP64 封装无硬件 TIM 通道，必须用 Arduino Servo 库（不能用原 C06B 的寄存器操作法）
- V3.0 CH340N DTR/RTS 未接 NRST/BOOT0，不支持自动下载，需手动 BOOT0+RESET
- MPU9250 在此板上走 SPI（非 I2C），WHO_AM_I=0x71 确认
- PB5 实为外接 RGB 灯带接口，非板载 LED；板载 MCU LED 在 PC13 (active-LOW)
- WFLY RF209S byte24 非标准 0x00，需放宽帧尾校验；failsafe 用 bit4(0x10)
- Servo.attach 会重置 GPIOC 寄存器，导致同端口 PC13 LED 被意外关闭
- 模式切换时 delay() 阻塞主循环会导致 USART2 RX 缓冲区溢出丢帧 → 改用非阻塞蜂鸣
- 编译资源: RAM 4.0% (1964/49152), Flash 8.4% (21980/262144)

## [0.1.0] - 2026-06-15

### Added
- Project skeleton: directory structure, README, LICENSE, .gitignore
- Architecture design: dual-layer (RDK X5 L2 + STM32 L1) with SBUS safety override
- Documentation placeholders: architecture, hardware setup, protocol spec, migration plan, safety design
- ROS2 package scaffold: `src/tracked_vehicle/` with module stubs
