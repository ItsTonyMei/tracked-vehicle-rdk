#pragma once
#include <Arduino.h>

// ============================================================================
// 履带车视觉跟随系统 — STM32 ROS V3.0 扩展板配置
// MCU: STM32F103RCT6 (Cortex-M3, 72MHz, 256KB Flash, 48KB SRAM)
// 功能: SBUS接收 + X5 MotorCmd解析 + 坦克混控 + ESC PWM + 安全超时
// ============================================================================

// ─── PWM 参数 (ZTW Seal G2 双路无刷电调, 50Hz) ───
// 1500μs 中位 / 1000-2000μs 范围 = 标准舵机 PWM 通用规范
// 1500μs 为标准舵机 PWM 中位, 详见 docs/lessons-learned.md
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
constexpr uint32_t SBUS_HDR_GAP_US     = 1000;  // 帧头前最小空闲: 帧内字节~110μs 连续,
                                                // 帧间空闲~11ms → 1ms 阈值区分真假帧头 0x0F

// SBUS → PWM 映射 (WFLY 实测校准)
// 中位 raw=1024 (≠ FrSky 标准 992, CH6 三档实测 352/1024/1695),
// 满行程 raw ≈ 352-1695 → ±672. 中位 → PWM 1500, 满杆 → ±500μs 全行程.
constexpr int      SBUS_CENTER          = 1024;  // WFLY 摇杆中位 raw 值
constexpr int      SBUS_RAW_HALF_SPAN   = 672;   // 1024-352 = 1695-1024
constexpr int      SBUS_THR_SENSITIVITY = 500;   // 满杆输出偏移 (μs)
constexpr int      SBUS_STR_SENSITIVITY = 500;

// ─── 蜂鸣器 (PC5, 经 NPN 三极管 S8050, active-HIGH) ───
constexpr uint8_t  PIN_BUZZER          = PC5;

// ─── 板载 LED (PC13, active-LOW) ───
// 已验证: PC13 控制板载 MCU 红色 LED
// STM32 标准: PC13 为内置 LED，低电平点亮
constexpr uint8_t  PIN_LED             = PC13;
constexpr bool     LED_ACTIVE_LOW      = true;

// ─── 安全时序 ───
constexpr uint32_t ESC_INIT_DELAY_MS   = 3000;   // ESC 自检
constexpr uint32_t X5_FRESH_TIMEOUT_MS = 2000;   // X5 指令新鲜度: 超时 → 自动模式停车待命
constexpr uint32_t SBUS_LOCK_TIMEOUT_MS= 2000;   // 手控模式命令超时: 超时 → 自动锁定 (需重新 ARM)
// 注: PWM 斜率软启动 (SLEW_RATE_US_PER_S) 已于 2026-07-21 移除 — 用户要求
// 全速响应. 若电机浪涌导致 X5 欠压重启复发, 需恢复斜率限制 (见 git 历史)
constexpr uint32_t STATUS_INTERVAL_MS  = 200;    // 5Hz 状态输出
constexpr int      DIR_THRESHOLD       = 20;     // 方向判定阈值 (μs)

// ─── MotorCmd 协议 (X5 → STM32, 与 ESP32/ESP8266 一致) ───
// Frame: [0xAA][th_lo][th_hi][st_lo][st_hi][CRC8]  6 bytes @ 115200
constexpr uint8_t  MOTORCMD_HEADER     = 0xAA;
constexpr uint8_t  MOTORCMD_FRAME_LEN  = 6;

// ─── CRC8: 供 crc8() 使用 (main.cpp) ───
constexpr uint8_t  CRC8_POLY           = 0x07;
constexpr uint8_t  CRC8_INIT           = 0x00;

// ─── SBUS 通道映射 (WFLY RF209S) ───
// CH1=方向(右手左右), CH2=油门(左手上下), CH3=升降, CH4=方向舵
// CH5=ARM/DISARM (3帧防抖), CH6=手控RC/自动X5 (非阻塞蜂鸣)
constexpr uint8_t  SBUS_CH_STEERING    = 0;     // CH1
constexpr uint8_t  SBUS_CH_THROTTLE    = 1;     // CH2
constexpr uint8_t  SBUS_CH_ARM         = 4;     // CH5 (LOW=DISARM, HIGH=ARM)
constexpr uint8_t  SBUS_CH_MODE        = 5;     // CH6 (LOW=手控RC, HIGH=X5模式 RDK X5决策)
constexpr uint16_t SBUS_ARM_THRESHOLD  = 1024;  // CH5 > this = ARMED
// CH6 模式判定: 中值滤波 + Schmitt 滞回 (>1500 自动 / <600 手控), 见 main.cpp
// 模式切换稳定确认 (非对称): 切回手控=紧急接管通道需快, 切入X5可从容
constexpr uint32_t MODE_TO_MANUAL_MS   = 300;   // X5 → 手控: 300ms 稳定即切换
constexpr uint32_t MODE_TO_AUTO_MS     = 1000;  // 手控 → X5: 1s 稳定才切换
// 此裸阈值仅保留作参考, 逻辑中不再使用
constexpr uint16_t SBUS_MODE_THRESHOLD = 1024;

// ─── MPU9250 IMU (SPI2, PB12-PB15) ───
constexpr uint8_t  PIN_IMU_NSS         = PB12;
constexpr uint8_t  PIN_IMU_SCLK        = PB13;
constexpr uint8_t  PIN_IMU_MISO        = PB14;
constexpr uint8_t  PIN_IMU_MOSI        = PB15;
