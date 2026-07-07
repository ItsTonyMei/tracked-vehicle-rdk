# 通信协议定义

## 1. MotorCmd — X5 → STM32 下行

### 帧格式 (6 bytes, 115200 bps)

| Byte | 字段 | 说明 |
|------|------|------|
| 0 | `0xAA` | 帧头 |
| 1 | `throttle_lo` | 油门低字节 (uint16 LE) |
| 2 | `throttle_hi` | 油门高字节 |
| 3 | `steering_lo` | 转向低字节 (uint16 LE) |
| 4 | `steering_hi` | 转向高字节 |
| 5 | `CRC8` | 1-4 字节的 CRC-8/ITU |

- **CRC8**: poly=0x07, init=0x00, 与 ESP32/ESP8266/原项目一致
- **发送间隔**: 跟随 /cmd_vel topic 发布频率（body_tracking 约 30Hz）
- **范围**: throttle/steering 均为 1000-2000μs (uint16)
- **停止值**: throttle=1500, steering=1500
- **超时**: 2s 无有效帧 → STM32 自动切中位 + 蜂鸣锁定 (CMD_TIMEOUT_MS=2000)

### 坦克混控 (STM32 端)

```text
sOff = steering - 1500
left  = throttle + sOff    (钳位 1000-2000)
right = throttle - sOff    (钳位 1000-2000)
```

- 1500μs 为中位 (标准舵机 PWM)

### X5 端串口说明

X5 通过 Micro USB → CH340N → STM32 USART1 (PA9/PA10) 发送 MotorCmd。同一串口 STM32 也会输出 debug 文本 (5Hz 状态行)。debug 行不包含 `0xAA` 前缀，因此不会干扰 MotorCmd 解析。

---

## 2. SBUS — 遥控器 → STM32 下行

### 物理层

| 参数 | 值 |
|------|-----|
| 波特率 | 100000 |
| 数据位 | 8 |
| 校验 | Even Parity |
| 停止位 | 2 |
| 帧间隔 | 14ms (标准) / 7ms (高速) |
| 信号极性 | 经三极管反相后为标准 UART idle=HIGH |

### 帧格式 (25 bytes)

| Byte | 内容 |
|------|------|
| 0 | 帧头 `0x0F` |
| 1-22 | 16 通道 × 11 bits = 176 bits |
| 23 | 标志位: bit2=lost_frame, bit3=failsafe |
| 24 | 帧尾 `0x00` |

### 11-bit 通道解包

```text
ch[0]  = (buf[1]      | buf[2] << 8) & 0x07FF
ch[1]  = (buf[2] >> 3 | buf[3] << 5) & 0x07FF
ch[2]  = (buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF
... (按此模式解码全部 16 通道)
```

### 通道映射 (WFLY RF209S 遥控器)

| 通道 | 功能 | SBUS 索引 | 说明 |
|------|------|-----------|------|
| CH1 | 方向/转向 | channels[0] | 右手左右 |
| CH2 | 油门 | channels[1] | 左手上下 |
| CH3 | 升降 | channels[2] | 未用 |
| CH4 | 方向舵 | channels[3] | 未用 |
| CH5 | ARM/DISARM | channels[4] | LOW=锁定, HIGH=解锁 (3帧防抖) |
| CH6 | 手控/自动 | channels[5] | LOW=手控RC, HIGH=自动X5 |

### SBUS → PWM 映射

```text
SBUS 典型范围: 172 (min) ~ 992 (center) ~ 1811 (max)
死区: ±20 (约 ±5% 摇杆行程)
灵敏度: ±250μs (满杆偏移)
```

### 控制优先级 (含 CH6 模式)

```text
手控模式 (CH6=LOW):
  SBUS 遥控器 (CH5=ARMED)  ▸  最高优先, 摇杆直控
  X5 自主指令               ▸  次优先
  超时刹停 (2s)             ▸  安全兜底

自动模式 (CH6=HIGH):
  X5 自主指令               ▸  优先
  SBUS CH5=LOCK             ▸  可紧急刹停
  超时刹停 (2s)             ▸  安全兜底
```

### SBUS 信号防抖

- 信号生效: 连续 **5 帧** 有效 SBUS 帧 → `g_sbus.valid = true` (防悬空噪声)
- 信号丢失: 连续 **2 次** 超时 (200ms/次) → `g_sbus.valid = false` (防单帧丢帧误判)
- 信号丢失 → 立即 DISARM + 刹停
- 信号恢复 → 保持 DISARM, **需手动 CH5 LOW→HIGH 重新解锁**

### Failsafe

- WFLY RF209S: failsafe 标志位为 byte23 **bit4** (0x10), 非标准 bit3 (0x08)
- SBUS 帧 byte23 bit2=1 → lost_frame → 丢弃该帧
- failsafe 激活 → 立即切中位 1500μs

### WFLY RF209S 适配说明

- byte24 非标准 0x00 (变化值), 已放宽帧尾校验
- failsafe 标志位偏移 (bit4 vs 标准 bit3)
- 未使用通道可能输出极端低值 (~4), 不能用于噪声过滤

---

## 3. VIS 帧 — OpenMV → X5 (参考)

```text
格式: "VIS:cx,cy,w,h,feetY,conf,PERSON,distScore,tofDist*CRC8\r\n"
波特率: 4800 bps
校验: XOR checksum (payload 部分, 不含 VIS: 前缀)
```

用于后视辅助，解析逻辑在 X5 端 ROS2 节点实现。

---

## 4. 上行遥测 — STM32 → X5 (预留)

```text
[0xBB][EncL:2B][EncR:2B][CurL:1B][CurR:1B][Flags:1B][ErrCode:1B][CRC8:1B]
10 bytes
```

当前固件仅通过串口文本输出状态（5Hz），未来可通过此二进制帧上报编码器/电流数据。
