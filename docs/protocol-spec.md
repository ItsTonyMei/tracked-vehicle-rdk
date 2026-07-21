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
- **发送间隔**: 跟随 /cmd_vel 发布频率 (motion_arbiter 10Hz / body_tracking 30Hz), **motor_bridge keepalive 20Hz 持续重发保证 STM32 永不断流**
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
| 0 | 帧头 `0x0F` (v0.8.2: 前需 ≥1ms 空闲间隔 → 消除数据字节 0x0F 假帧) |
| 1-22 | 16 通道 × 11 bits = 176 bits |
| 23 | 标志位: bit2=lost_frame, bit3=failsafe (WFLY: bit4) |
| 24 | 帧尾 `0x00` (WFLY 非标, v0.8.2 已移除校验 → 靠空闲间隔帧同步保完整性) |

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
| CH5 | ARM/DISARM | channels[4] | LOW=锁定, HIGH=解锁 (帧同步3帧 ~42ms 防抖, v0.8.2) |
| CH6 | 手控/自动 | channels[5] | Schmitt滞回 (>1500自动/<600手控) + 非对称延时 (→手控300ms/→X5 1s), v0.8.2 |

### SBUS → PWM 映射 (WFLY 实测校准, v0.8.2)

```text
WFLY RF209S 实测: raw 352 (min) ~ 1024 (center) ~ 1695 (max)
  ≠ FrSky 标准 172~992~1811 — 不同接收机 raw 中位不同, 必须以实测校准!
死区: ±20 raw (约 ±15μs 输出)
映射: PWM 1500 + (raw - 1024) × 500 / 672 → 满杆精确 ±500μs 全行程
诊断: 状态行输出 c1=/c2= SBUS 原始值, 现场可验证
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
- **(v0.8.2) 帧同步层**: 0x0F 帧头前需 ≥1ms 空闲 → 消除通道数据 0x0F 导致的错位锁定
- **(v0.8.2) 通道滤波**: CH5 帧同步 3 帧 (~42ms) + CH6 5 帧中值 (~70ms) + Schmitt 滞回
- **(v0.8.2) 诊断**: 状态行 ore= (ORE 物理丢帧) / hdr= (假帧头拒绝) 计数器, 现场定位噪声源
- **(v0.8.2) 错误处理**: ORE (数据溢出) → 整帧丢弃; FE/NE/PE (线路噪声) → 仅清标志, 信道数据仍然可用

### Failsafe

- WFLY RF209S: failsafe 标志位为 byte23 **bit4** (0x10), 非标准 bit3 (0x08)
- SBUS 帧 byte23 bit2=1 → lost_frame → 丢弃该帧
- failsafe 激活 → 立即切中位 1500μs

### WFLY RF209S 适配说明

- **raw 中位 = 1024** (≠ FrSky 992), CH6 三档实测 352/1024/1695, 摇杆 full-range 一致 → SBUS_CENTER=1024
- byte24 非标准 0x00 (变化值), **v0.8.2 已移除帧尾校验 → 改用空闲间隔帧头校验** (帧间空闲 ~11ms, 帧内字节~110μs 连续, ≥1ms 阈值区分)
- failsafe 标志位偏移 (bit4=0x10 vs 标准 bit3=0x08)
- lost_frame (byte23 bit2) → 等同 failsafe 处理
- 未使用通道可能输出极端低值 (~4), 不能用于噪声过滤
- **关键教训**: 换接收机/遥控器必须实测 raw 值校准, 不可假设"标准"中位

---

## 3. VIS 帧 — OpenMV → X5 (参考)

```text
格式: "VIS:cx,cy,w,h,feetY,conf,PERSON,distScore,tofDist*CRC8\r\n"
波特率: 4800 bps
校验: XOR checksum (payload 部分, 不含 VIS: 前缀)
```

用于后视辅助，解析逻辑在 X5 端 ROS2 节点实现。

---

## 4. CI1302 A5 FA — X5 ↔ 语音模块

### 帧格式 (8 bytes, 115200 bps)

| Byte | 字段 | 说明 |
|------|------|------|
| 0 | `0xA5` | 帧头 1 |
| 1 | `0xFA` | 帧头 2 |
| 2 | `0x00` | 保留 |
| 3 | `TYPE` | `0x81`=模块→X5 (识别), `0x82`=X5→模块 (播报) |
| 4 | `CMD` | 命令 ID (见下表) |
| 5 | `0x00` | 保留 |
| 6 | `CKSUM` | `(A5+FA+00+TYPE+CMD+00) & 0xFF` |
| 7 | `0xFB` | 帧尾 |

### 命令词协议表 (14 条, V01843 固件)

格式: `命令词 : 识别帧(模块→X5) : 应答帧(X5→模块)`

| 命令词 | CMD | 识别帧 (模块→X5) | 应答帧 (X5→模块) |
|--------|-----|-------------------|-------------------|
| 你好瓦力 | 0x01 | `A5 FA 00 81 01 00 21 FB` | `A5 FA 00 82 01 00 22 FB` |
| `<欢迎语>` | 0x02 | — | `A5 FA 00 82 02 00 23 FB` |
| `<休息语>` | 0x03 | — | `A5 FA 00 82 03 00 24 FB` |
| 增大音量 | 0x04 | `A5 FA 00 81 04 00 24 FB` | `A5 FA 00 82 04 00 25 FB` |
| 减小音量 | 0x05 | `A5 FA 00 81 05 00 25 FB` | `A5 FA 00 82 05 00 26 FB` |
| 小车停车/停止 | 0x06 | `A5 FA 00 81 06 00 26 FB` | `A5 FA 00 82 06 00 27 FB` |
| 小车前进 | 0x07 | `A5 FA 00 81 07 00 27 FB` | `A5 FA 00 82 07 00 28 FB` |
| 小车后退 | 0x08 | `A5 FA 00 81 08 00 28 FB` | `A5 FA 00 82 08 00 29 FB` |
| 小车左转 | 0x09 | `A5 FA 00 81 09 00 29 FB` | `A5 FA 00 82 09 00 2A FB` |
| 小车右转 | 0x0A | `A5 FA 00 81 0A 00 2A FB` | `A5 FA 00 82 0A 00 2B FB` |
| 小车左旋 | 0x0B | `A5 FA 00 81 0B 00 2B FB` | `A5 FA 00 82 0B 00 2C FB` |
| 小车右旋 | 0x0C | `A5 FA 00 81 0C 00 2C FB` | `A5 FA 00 82 0C 00 2D FB` |
| 跟我走/开启跟随 | 0x0D | `A5 FA 00 81 0D 00 2D FB` | `A5 FA 00 82 0D 00 2E FB` |
| 别跟我/关闭跟随 | 0x0E | `A5 FA 00 81 0E 00 2E FB` | `A5 FA 00 82 0E 00 2F FB` |

> 每个 CMD 支持多个中文命令词 (如 停止=停车=停止前进).
> motion_arbiter 仅处理 CMD 0x06-0x0E (运动+跟随), 音量/唤醒由模块端固件管理.
> 完整固件及刷机工具见 `ci1302_firmware/sfw*/`.

### 固件

- 版本: CI1302 V01843 (内部RC + 关闭波特率校准)
- 唤醒词: 独立 DNN 模型门控 (`USE_SEPARATE_WAKEUP_EN=1`)
- 文件: `ci1302_firmware/CI1302_chinese_1mic_V01843_UART1_115200_2M.bin`

### 实现参考

X5 端节点: `motion_arbiter.py` (`_TYPE_FROM_CI1302=0x81`, `_TYPE_TO_CI1302=0x82`)

---

## 5. PWM 输出 — STM32 → 电调

| 参数 | 值 |
|------|-----|
| 方式 | Arduino Servo 库软件 PWM |
| 频率 | 50Hz (周期 20000μs) |
| 中位 | 1500μs (标准舵机 PWM 规范, 非1257/1275等非标值) |
| 最小 | 1000μs |
| 最大 | 2000μs |
| 左电机 | S1 — PC3 |
| 右电机 | S2 — PC2 |

### 坦克混控

```
sOff = steering - 1500
left  = throttle + sOff    (钳位 1000-2000)
right = throttle - sOff    (钳位 1000-2000)
```

---

## 6. 上行遥测 — STM32 → X5 (预留)

```text
[0xBB][EncL:2B][EncR:2B][CurL:1B][CurR:1B][Flags:1B][ErrCode:1B][CRC8:1B]
10 bytes
```

当前固件仅通过串口文本输出状态（5Hz），未来可通过此二进制帧上报编码器/电流数据。
