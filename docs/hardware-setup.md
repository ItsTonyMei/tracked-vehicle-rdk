# 硬件连线与接口定义

## STM32 ROS V3.0 扩展板引脚表

**MCU**: STM32F103RCT6 (Cortex-M3, 72MHz, 256KB Flash, 48KB SRAM, LQFP64)
**USB-UART**: CH340N (Micro USB)
**IMU**: MPU9250 (SPI1)

### 引脚功能表

| 功能 | 引脚 | 外设 | 说明 |
|------|------|------|------|
| X5 通信 TX | PA9 | USART1_TX | CH340N → Micro USB → X5 |
| X5 通信 RX | PA10 | USART1_RX | CH340N → Micro USB → X5 |
| SBUS 输入 | PA3 | USART2_RX | 经三极管反相, 100000bps 8E2 |
| 左电调 PWM | PC3 | S1 (Servo) | ZTW Seal G2, 50Hz, 650-1900μs |
| 右电调 PWM | PC2 | S2 (Servo) | ZTW Seal G2, 50Hz, 650-1900μs |
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

> ZTW Seal G2 中位 1275μs 为非标值 (不是标准舵机 1500μs)，已在 C06B 板上实测验证。

### SBUS 接线

```
WFLY RF209S SBUS 输出 ─── V3.0 板 SBUS 排针 (USART2 PA3, 经板载三极管反相)
```

SBUS 信号本身是反相 UART (idle low)，经板载 NPN 三极管反相后变为标准 UART (idle high) 送入 STM32 USART2。

### 烧录

| 参数 | 值 |
|------|-----|
| 工具 | PlatformIO + stm32flash |
| 端口 | COM6 (CH340N) |
| 波特率 | 115200 |
| 流程 | 手动 BOOT0 → RESET → 松 BOOT0 → `pio run -t upload` |

V3.0 板的 CH340N 仅接了 TX/RX (USART1)，**DTR/RTS 未连接到 NRST/BOOT0**，不支持自动下载。必须手动进 bootloader。

## 踩坑记录

1. **PWM 引脚**: PC0-PC3 在 LQFP64 封装上无硬件 TIM 通道，必须用 Arduino Servo 库。不能用原 C06B 的 TIM3 寄存器操作法。
2. **Serial 初始化**: `genericSTM32F103RC` 变体默认不映射 `Serial` 到 USART1，需在 `setup()` 中显式调用 `Serial.setRx(PA10); Serial.setTx(PA9);`
3. **IMU 接口**: MPU9250 在此板上走 SPI (不是常见 I2C)，WHO_AM_I 读回 0x71 确认芯片在线。
4. **SBUS 反相**: 板载三极管已将 SBUS 反相，USART2 配置为标准模式 (不开启 RXINV 位)。
5. **自动下载不可用**: V3.0 的 CH340N DTR/RTS 引脚悬空，所有时序组合均无效。手动 BOOT0+RESET 是唯一方式。与 C06B 不同（C06B 的 CH9102 DTR/RTS 有自动下载电路）。
