# 硬件连线与接口定义

## STM32 ROS V3.0 扩展板引脚表

**MCU**: STM32F103RCT6 (Cortex-M3, 72MHz, 256KB Flash, 48KB SRAM, LQFP64)
**USB-UART**: CH340N (Micro USB)
**IMU**: MPU9250 (SPI2, PB12-PB15)

### 引脚功能表

| 功能 | 引脚 | 外设 | 说明 |
|------|------|------|------|
| X5 通信 TX | PA9 | USART1_TX | CH340N → Micro USB → X5 |
| X5 通信 RX | PA10 | USART1_RX | CH340N → Micro USB → X5 |
| SBUS 输入 | PA3 | USART2_RX | 经三极管反相, 100000bps 8E2 |
| 左电调 PWM | PC3 | S1 (Servo) | ZTW Seal G2, 50Hz, 1000-2000μs |
| 右电调 PWM | PC2 | S2 (Servo) | ZTW Seal G2, 50Hz, 1000-2000μs |
| 蜂鸣器 | PC5 | GPIO | NPN S8050 驱动, active-HIGH |
| RGB 灯带 | PB5 | GPIO | 外接 RGB 灯带接口 |
| 板载 LED | PC13 | GPIO | MCU 红色 LED, active-LOW, 已测试可用 |
| USB LED | — | CH340N TX | USB 通信指示灯 (绿), 不可 GPIO 控制 |
| IMU NSS | PB12 | SPI1_NSS | MPU9250 片选 |
| IMU SCLK | PB13 | SPI1_SCK | MPU9250 时钟 |
| IMU MISO | PB14 | SPI1_MISO | MPU9250 数据回读 |
| IMU MOSI | PB15 | SPI1_MOSI | MPU9250 数据发送 |

### 电调接线

```
STM32 S1 (PC3) ─── ZTW Seal G2 左电调 信号线 (白/黄)
STM32 S2 (PC2) ─── ZTW Seal G2 右电调 信号线 (白/黄)
STM32 GND     ─── 两路电调 GND (共地)
                  
电调 BEC 5V 输出  ─── 悬空不接! (MCU 独立供电)
```

> **ZTW Seal G2 电调 PWM**: 1500μs 中位 / 1000-2000μs 范围为标准舵机 PWM 通用规范。原 C06B 项目批次误用非标 1275μs 中位，此处已修正。新电调批次以手册实测为准。

### SBUS 接线

```
WFLY RF209S SBUS 输出 ─── V3.0 板 SBUS 排针 (USART2 PA3, 经板载三极管反相)
```

SBUS 信号本身是反相 UART (idle low)，经板载 NPN 三极管反相后变为标准 UART (idle high) 送入 STM32 USART2。

### 烧录

工具: PlatformIO + stm32flash. 端口和流程详见 `stm32_firmware/flash_stm32.sh` 及 `stm32_firmware/platformio.ini`.
V3.0 CH340N DTR/RTS 未接 NRST/BOOT0, 不支持自动下载, 需手动 BOOT0→RESET→松BOOT0 进 bootloader.

### 板载 LED 一览

| 丝印 | 颜色 | 引脚 | 可控性 |
|------|------|------|--------|
| POWER | 红 | 3.3V 电源轨 | 常亮, 不可控 |
| 5V | 蓝 | 5V 电源轨 | 常亮, 不可控 |
| 6V8 | 蓝 | 6.8V 电源轨 | 常亮, 不可控 |
| MCU | 红 | PC13 (active-LOW) | 已启用, 快/中/慢三速闪烁 |
| USB | 绿 | CH340N TX | 通信时闪, 不可控 |

## 踩坑记录

硬件相关踩坑统一记录在 **[lessons-learned.md](lessons-learned.md)** 及 `stm32_firmware/src/main.cpp` 注释中，此处不重复列出。
