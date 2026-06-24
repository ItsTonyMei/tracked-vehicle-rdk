#pragma once
#include <Arduino.h>

// ============================================================================
// 履带车视觉跟随系统 — STM32 ROS V3.0 扩展板配置
// MCU: STM32F103RCT6 (Cortex-M3, 72MHz, 256KB Flash, 48KB SRAM)
// 功能: SBUS接收 + X5 MotorCmd解析 + 坦克混控 + ESC PWM + 安全超时
// ============================================================================

// ─── PWM 参数 (ZTW Seal G2 双路无刷电调, 50Hz) ───
// 1500μs 中位 / 1000-2000μs 范围 = 标准舵机 PWM 通用规范
// 踩坑: 原 C06B 项目电调批次误用非标 1275μs 中位,
//       1500μs 才是通用标准值, 非批次差异, 不同批次均应以标准值为准
constexpr uint16_t PWM_NEUTRAL         = 1500;
constexpr uint16_t PWM_MIN             = 1000;
constexpr uint16_t PWM_MAX             = 2000;

// ─── 舵机 PWM 引脚 (Servo 库软件 PWM) ───
// PC0-PC3 在 LQFP64 封装无硬件 TIM 通道, 使用 Servo 库
// S1=PC3→左电调, S2=PC2→右电调
constexpr uint8_t  PIN_ESC_LEFT        = PC3;   // S1
constexpr uint8_t  PIN_ESC_RIGHT       = PC2;   // S2

// ─── X5 通信 (USART1 PA9/PA10 → CH340N → Micro USB) ───
// 与 Serial debug 共享同一 USART1
constexpr uint32_t X5_BAUD             = 115200;

// ─── SBUS 接收 (USART2 PA3, 经三极管反相) ───
// WFLY RF209S 接收机 → 三极管反相器 → STM32 PA3 (USART2_RX)
// SBUS 协议: 100000 baud, 8E2, 25 bytes/frame @ 14ms (7ms 高速)
constexpr uint8_t  PIN_SBUS_RX         = PA3;
constexpr uint32_t SBUS_BAUD           = 100000;
constexpr uint8_t  SBUS_FRAME_LEN      = 25;
constexpr uint32_t SBUS_TIMEOUT_MS     = 200;   // 超时视为断开 (14ms×14帧的容错)

// SBUS → PWM 灵敏度 (满杆=中位 ±250μs, 保守安全)
constexpr int      SBUS_THR_SENSITIVITY = 250;
constexpr int      SBUS_STR_SENSITIVITY = 250;

// ─── 蜂鸣器 (PC5, 经 NPN 三极管 S8050, active-HIGH) ───
constexpr uint8_t  PIN_BUZZER          = PC5;

// ─── 板载 LED (PC13, active-LOW) ───
// 已验证: PC13 控制板载 MCU 红色 LED
// STM32 标准: PC13 为内置 LED，低电平点亮
constexpr uint8_t  PIN_LED             = PC13;
constexpr bool     LED_ACTIVE_LOW      = true;

// ─── 安全时序 ───
constexpr uint32_t ESC_INIT_DELAY_MS   = 3000;   // ESC 自检
constexpr uint32_t CMD_TIMEOUT_MS      = 2000;   // 2s 无命令 → 锁定 (X5 指令间隔 50ms)
constexpr uint32_t STATUS_INTERVAL_MS  = 200;    // 5Hz 状态输出
constexpr int      DIR_THRESHOLD       = 20;     // 方向判定阈值 (μs)

// ─── MotorCmd 协议 (X5 → STM32, 与 ESP32/ESP8266 一致) ───
// Frame: [0xAA][th_lo][th_hi][st_lo][st_hi][CRC8]  6 bytes @ 115200
constexpr uint8_t  MOTORCMD_HEADER     = 0xAA;
constexpr uint8_t  MOTORCMD_FRAME_LEN  = 6;

// ─── CRC8 (poly 0x07, init 0x00) ───
constexpr uint8_t  CRC8_POLY           = 0x07;
constexpr uint8_t  CRC8_INIT           = 0x00;

// ─── SBUS 通道映射 (WFLY RF209S) ───
// CH1=方向(右手左右), CH2=油门(左手上下), CH3=升降, CH4=方向舵
// CH5=ARM/DISARM (3帧防抖), CH6=手控RC/自动X5 (非阻塞蜂鸣)
constexpr uint8_t  SBUS_CH_STEERING    = 0;     // CH1
constexpr uint8_t  SBUS_CH_THROTTLE    = 1;     // CH2
constexpr uint8_t  SBUS_CH_ARM         = 4;     // CH5 (LOW=DISARM, HIGH=ARM)
constexpr uint8_t  SBUS_CH_MODE        = 5;     // CH6 (LOW=手控RC, HIGH=自动X5)
constexpr uint16_t SBUS_ARM_THRESHOLD  = 1024;  // CH5 > this = ARMED
constexpr uint16_t SBUS_MODE_THRESHOLD = 1024;  // CH6 > this = 自动模式

// ─── MPU9250 IMU (SPI2, PB12-PB15) ───
constexpr uint8_t  PIN_IMU_NSS         = PB12;
constexpr uint8_t  PIN_IMU_SCLK        = PB13;
constexpr uint8_t  PIN_IMU_MISO        = PB14;
constexpr uint8_t  PIN_IMU_MOSI        = PB15;
