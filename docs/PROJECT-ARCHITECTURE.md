# 项目架构与研发历程

## 1. 项目概述

**6WD 重型履带车 RDK X5 自主跟随系统** — 基于地平线 RDK X5 开发板的 ROS2 机器人，通过语音+手势控制实现对特定人物的自动跟随。

### 硬件清单

| 组件 | 型号 | 用途 |
|------|------|------|
| 主控 | RDK X5 (地平线 Sunrise 5) | TROS/ROS2 运行时, BPU AI 推理 (8 TOPS) |
| 摄像头 | GS130W (SC132GS, 1.75mm 广角) | 960×544 @ 60fps, 人体/手势检测 |
| 激光雷达 | YDLidar T-mini Plus | 360° @ 10Hz, 430pts, 测距融合 |
| 下位机 | STM32F103RCT6 (V3.0 扩展板) | SBUS 遥控器 + MotorCmd → ESC PWM |
| 语音模块 | CI1302 (V01843 固件) | 离线中文语音识别 → UART 命令 |
| 车体 | 6WD 重型履带车 | ZTW Seal G2 ESC, 1500μs 中位 PWM |

### 传感器安装

```
安装高度 (~150cm, 胸/头部水平):
  Camera: 5cm 前, 20cm 高 (base_link → camera_frame)
  LiDAR:  0cm 前, 2cm 高 (base_link → laser_frame)
  
LiDAR 扫描平面 ≈ 人体胸部高度, 扫描到的是躯干连续弧面 (非腿部双柱)
Camera 72° HFOV (GDC 去畸变后), LiDAR 360° 全覆盖
```

---

## 2. 控制层级 (5 层安全金字塔)

```
Layer 1: RC CH5 油门锁     ← 物理安全开关, 0=强制停车, 1=解锁
Layer 2: RC CH6 X5 模式    ← 上位机接管: Schmitt滞回 (>1500自动/<600手控), 非对称延时 (→手控300ms/→X5 1s)
Layer 3: 语音手动模式       ← VOICE_MANUAL: 前进/后退/转向/停止语音命令直接驱动
Layer 4: 语音跟随模式       ← FOLLOWING: "开启跟随" → 自动追踪画面中人体
Layer 5: 手势锁定目标       ← OK=锁定特定人物, Palm=解除锁定
```

每层是上一层的前置条件。RC 油门锁可以在任意时刻物理切断动力。

---

## 3. ROS2 节点图 (11 节点数据流)

```
┌─────────────────────────────────────────────────────────────────┐
│                        SENSOR LAYER                              │
│                                                                  │
│  [mipi_cam]          [ydlidar]          [CI1302 语音]            │
│  GS130W 60fps        360° 10Hz          UART A5 FA 协议           │
│  960×544 NV12        430pts             中文离线识别               │
│      │                   │                   │                   │
│      ▼                   │                   │                   │
│  [hobot_codec]           │                   │                   │
│  NV12→JPEG               │                   │                   │
│  /image                  │                   │                   │
│      │                   │                   │                   │
├──────┼───────────────────┼───────────────────┼───────────────────┤
│      │          AI LAYER │                   │                   │
│      ▼                   │                   │                   │
│  [mono2d_body_det]       │                   │                   │
│  BPU 推理 60fps           │                   │                   │
│  body/head/face/hand     │                   │                   │
│  /hobot_mono2d_body_     │                   │                   │
│   detection              │                   │                   │
│      │                   │                   │                   │
│      ├──► [hand_lmk]     │                   │                   │
│      │    手部21点关键点  │                   │                   │
│      │    /hobot_hand_    │                   │                   │
│      │    lmk_detection   │                   │                   │
│      │        │           │                   │                   │
│      │        ▼           │                   │                   │
│      ├──► [hand_gesture]  │                   │                   │
│      │    OK/Palm 分类    │                   │                   │
│      │    /hobot_hand_    │                   │                   │
│      │    gesture_det     │                   │                   │
│      │        │           │                   │                   │
│      │        ▼           │                   │                   │
│      │  [body_tracking]   │                   │                   │
│      │  跟随策略+启动跟踪  │                   │                   │
│      │  /cmd_vel_body_    │                   │                   │
│      │  track             │                   │                   │
│      │        │           │                   │                   │
├──────┼────────┼───────────┼───────────────────┼───────────────────┤
│      │  FUSION LAYER      │                   │                   │
│      │        │           ▼                   │                   │
│      │        │      [perception_node] ◄──────┘                   │
│      │        │      单一权威融合源           ◄── /scan           │
│      │        │      • LiDAR-Camera 融合 (自适应聚类+躯干过滤     │
│      │        │        +匈牙利匹配+EKF)                           │
│      │        │      • 手势锁定 (OK=锁定, Palm=解除)              │
│      │        │      • 障碍物急停 (<0.5m, ±15°)                  │
│      │        │      • HDMI 屏显 (1024×600)                       │
│      │        │                                                   │
│      │        │  ──/locked_target (Point: 距离+侧向)──►           │
│      │        │  ──/locked_track_id (Int32)──────────►           │
│      │        │  ──/emergency_stop (Bool)────────────►           │
│      │        │                                                   │
├──────┼────────┼───────────────────────────────────────────────────┤
│      │  CONTROL LAYER                                             │
│      │        │                                                   │
│      │        ▼              ◄── CI1302 UART (语音命令)          │
│      │  [motion_arbiter]                                          │
│      │  /cmd_vel 唯一发布者                                       │
│      │  • VOICE_MANUAL: 语音直接驱动                              │
│      │  • FOLLOWING:    LiDAR距离→线速度 连续映射                 │
│      │                  LiDAR侧向→角速度 (k=0.5 rad/s/m)          │
│      │  • 急停:         障碍物→零速                               │
│      │       │                                                   │
│      │       ▼                                                   │
├──────┼────── [motor_bridge] ──────────────────────────────────────┤
│      │  EXEC LAYER    /cmd_vel→MotorCmd                           │
│      │  [0xAA][th_lo][th_hi][st_lo][st_hi][CRC8] @ 115200       │
│      │       │                                                   │
│      │       ▼                                                   │
│      │  UART /dev/stm32_board → STM32 → ESC PWM → 履带车         │
└──────┴───────────────────────────────────────────────────────────┘
```

---

## 4. 各节点详解

### 4.1 SENSOR LAYER (传感器层)

#### mipi_cam — 摄像头驱动
- **输入**: SC132GS sensor (1280×1088 native, 1.75mm 广角 lens)
- **输出**: 960×544 NV12 @ 60fps via shared memory
- **处理**: rotation=90°, GDC 去畸变, calibration from `sc132gs_calibration_90.yaml`
- **设计原因**: SC132GS 是竖屏 sensor, rotation=90 转为横屏; GDC 校正在 BPU 硬件加速

#### ydlidar_ros2_driver — 激光雷达驱动
- **输入**: UART `/dev/ydlidar` @ 230400 bps
- **输出**: `/scan` (LaserScan), 360° @ 10Hz, 430pts, 0.84°/pt, 0-12m range
- **配置**: `fixed_resolution: true` → 输出固定 430 点 (非原始采样率)
- **设计原因**: T-mini Plus 通过三角测距, 10Hz 扫描频率适合人体跟随 (100ms 延迟可接受)

#### CI1302 语音模块
- **协议**: A5 FA 帧, 8 bytes @ 115200, V01843 固件
- **命令**: 前进(0x07)/后退(0x08)/左转(0x09)/右转(0x0A)/停止(0x06)/开启跟随(0x0D)/关闭跟随(0x0E)
- **设计原因**: 离线中文识别 (无需网络), 独立 MCU 处理, 不占 X5 CPU

### 4.2 AI LAYER (地平线 BPU 加速推理)

#### mono2d_body_detection — 多任务人体检测
- **模型**: `multitask_body_head_face_hand_kps_960x544.hbm`
- **输出**: `/hobot_mono2d_body_detection`, 30fps (image_gap=1 → 60fps 全员恢复)
- **包含**: body/head/face/hand ROI + keypoints, 每人 4 个独立 target
- **设计原因**: 单模型多任务 (比分别跑 body_det + hand_det 节省 50% BPU)

#### hand_lmk_detection — 手部关键点
- **模型**: `handLMKs.hbm`, 21 点手部关键点
- **输出**: `/hobot_hand_lmk_detection`
- **设计原因**: 为 hand_gesture_detection 提供输入特征

#### hand_gesture_detection — 手势分类
- **模型**: `gestureDet_8x21.hbm`, 静态手势检测
- **输出**: `/hobot_hand_gesture_detection`, 属性码 OK=11 / Palm=5
- **设计原因**: 使用手势而非物理按钮做目标锁定, 用户无需接触车辆

#### body_tracking — 跟随策略
- **输出**: `/cmd_vel_body_track` (Twist)
- **功能**: bbox 居中旋转 → angular.z, 人体距离→ linear.x (视觉估计)
- **参数**: `activate_wakeup_gesture=0` (手势由 perception_node 处理, 不启用内置唤醒)
- **设计原因**: 地平线官方跟随算法, 将 bbox 追踪转换为速度命令

### 4.3 FUSION LAYER (融合层)

#### perception_node — 感知权威节点

这是系统的**单一权威数据源**, 所有高层决策信息由此节点产生。

**订阅**:
- `/image` → 渲染画面
- `/hobot_mono2d_body_detection` → 人体检测结果
- `/hobot_hand_gesture_detection` → 手势分类
- `/scan` → LiDAR 点云
- `/follow_active` → 跟随模式状态

**发布**:
- `/locked_target` (Point: x=距离, y=侧向偏移) → motion_arbiter 控制速度
- `/locked_track_id` (Int32) → 锁定的行人 ID
- `/emergency_stop` (Bool) → 前方障碍物急停
- `/system_ready` (Bool) → 启动就绪信号

**核心算法**:
1. **LiDAR-Camera 融合**:
   - 自适应距离聚类 (近 0.10m ↔ 远 0.40m)
   - 躯干几何过滤 (弧宽 15-70cm + 曲率 <0.97)
   - 匈牙利角度匹配 (scipy linear_sum_assignment)
   - EKF 状态估计 [x, y, vx, vy], Q×dt 缩放
2. **手势锁定**: OK=11/Palm=5 属性码投票 (15 帧, ~0.25s @ 60fps)
3. **障碍物急停**: 前方 ±15° 内 <0.5m → /emergency_stop
4. **HDMI 渲染**: OpenCV 全屏 1024×600, 系统状态栏 (CPU/BPU/MEM/TEMP/FPS)

### 4.4 CONTROL LAYER (控制层)

#### motion_arbiter — 运动仲裁节点

`/cmd_vel` 的**唯一发布者**, 消除多写冲突。

**状态机**: VOICE_MANUAL ↔ FOLLOWING

**VOICE_MANUAL 模式**: 语音命令直接发布速度指令, 3s 超时自动 STOP

**FOLLOWING 模式**: 复合控制:
- **线速度**: LiDAR 融合距离 → 连续速度映射
  - <0.70m: -0.3 m/s (后退)
  - 0.70-0.85m: -0.3→0 过渡 (15cm 消除震颤)
  - 0.85-1.2m: 0 (合适范围)
  - 1.2-3.0m: 0→0.8 m/s (二次曲线加速)
  - ≥3.0m: 0.8 m/s (全速)
- **角速度**: LiDAR 侧向偏移 (k=0.5 rad/s/m, 优先) 或 bbox 像素居中 (fallback)
- **急停**: /emergency_stop → 纯零速 Twist

**设计原因**: 
- 单一 `/cmd_vel` 发布者避免多节点竞争控制权
- LiDAR 距离比 bbox 视觉估计精确 (度量值 vs 像素启发式)
- 语音命令在 FOLLOWING 模式下仍可临时接管 (3s 后自动恢复跟随)

### 4.5 EXEC LAYER (执行层)

#### motor_bridge — 串口桥接
- **输入**: `/cmd_vel` (Twist)
- **输出**: 6 字节 MotorCmd `[0xAA][th_lo][th_hi][st_lo][st_hi][CRC8]` @ 115200 → `/dev/stm32_board`
- **映射** (v0.9.0 校准增益): throttle=1500+linear.x×**1000**, steering=1500−angular.z×**450**
- **看门狗**: 60s 无命令 /cmd_vel → 自动发送 STOP
- **Keepalive**: 20Hz 重发最后命令, 防 STM32 判超时造成走走停停; 500ms 无新 cmd_vel 后自觉切中位 keepalive (STMCU 持续 fresh 不锁)
- **STM32 日志转发**: 读取 STM32 调试打印, 关键事件 ([SAFE]/[X5]/[MODE]/[SBUS]/ARM/DISARM/[DBG]/启动 banner 等) + 5Hz 状态行 (含 PWM 值/ore/hdr/c1/c2) 转发到 ROS 日志
- **设计原因**: 纯执行节点, 零决策; 解耦 ROS2 消息与 STM32 二进制协议

#### STM32 V3.0 固件 (v0.8.2)

- **输入**: MotorCmd (UART1) + SBUS (UART2, WFLY RF209S 接收机, **raw 中位=1024** ≠ FrSky 992)
- **CH5**: 油门锁 (连续3帧帧同步防抖, ~42ms确认) — 任意模式下**唯一的锁定手段**
- **CH6**: 模式切换: 5帧中值滤波(~70ms) + Schmitt滞回 (>1500自动/<600手控) + 非对称稳定确认 (→手控300ms紧急接管要快, →X5 1s)
- **输出**: 2× ESC PWM (S1=PC3 左, S2=PC2 右, 1500μs 中位), **满杆全行程 ±500μs (1000-2000)**, WFLY 精确校准 (SBUS_CENTER=1024, 量程 352-1695)
- **坦克混控**: throttle±steering → 左右电机差速
- **SBUS 帧同步**: 帧头 0x0F 前需 ≥1ms 空闲间隔 (帧内字节~110μs连续, 帧间~11ms — 消除数据字节 0x0F 错位锁定) + ORE 丢字节检测
- **2s 命令超时** (v0.8.1 起, **v0.8.2 修复 uint32 时间戳下溢**): 手控模式超时 → 自动锁定 (需重新 ARM); X5 模式超时 → 输出中位停车待命, **不锁定**, 指令恢复即继续
- **斜率软启动** (v0.8.1, **v0.8.2 已按用户决策移除**): 追求全速响应, 欠压风险靠供电侧解决
- **诊断**: 5Hz 状态行含 `ore=`(物理丢帧)/`hdr=`(假帧头拒绝)/`c1=`/`c2=`(SBUS 原始值), 经 motor_bridge 转发 ROS

---

## 5. 研发历程 (v0.1 → v0.9)

### Phase 1: 基础能力 (v0.1 - v0.4)

| 里程碑 | 内容 |
|--------|------|
| STM32 固件 | SBUS 解析 + MotorCmd → ESC PWM 坦克混控, CH5/CH6 安全门控 |
| M5 distScore | 地平线官方人体跟随节点, 基于 bbox 宽度的距离估计 |
| 本地屏显 | OpenCV 全屏渲染, 解决 rotation=90/坐标系/JPEG 解码等问题 |
| 系统精简 | 去除 xubuntu-desktop/gnome-shell/lightdm, 清出 1.3GB |

### Phase 2: 手势 + 语音 (v0.5)

| 里程碑 | 内容 |
|--------|------|
| 手势锁定 | OK=11/Palm=5, 30 帧投票 + 空间匹配 (手→人体关联) |
| 语音控制 | CI1302 V01843 固件 + voice_bridge 节点, 14 条命令 |
| 状态机 | VOICE_MANUAL ↔ FOLLOWING, voice_bridge 仲裁 /cmd_vel |
| 启动增强 | 进度条 + 逐项检测 + 超时报警, /system_ready 事件驱动 |

### Phase 3: LiDAR 融合 (v0.6 - 0.7)

| 里程碑 | 内容 |
|--------|------|
| LiDAR 集成 | YDLidar T-mini Plus, Euclidean 聚类 + 角度匹配 + EKF |
| 架构重构 | 三层分离: perception(融合)/motion_arbiter(仲裁)/motor_bridge(执行) |
| 距离覆写 | FOLLOWING 模式下 LiDAR 距离替代 bbox 视觉距离 |
| 资源优化 | image_gap=2 (30fps), 渲染 30Hz, 轮询 5Hz |

### Phase 4: 算法深化 (v0.8)

| 里程碑 | 内容 |
|--------|------|
| 手势修复 | image_gap 回退 (60fps), 投票 30→15, 纯属性码通道 |
| LiDAR 优化 | 自适应聚类, 躯干几何过滤, 匈牙利匹配, EKF Q×dt |
| 运动控制 | 速度连续映射, vel_fast 0.8 m/s, LiDAR 侧向转向 |
| 安全增强 | 障碍物急停, /emergency_stop topic |
| 代码清理 | 3 agent 并行审计, 移除 12 处死代码, 修复急停 angular 泄漏 |

### Phase 5: 固件攻坚 (v0.8.2)

| 里程碑 | 内容 |
|--------|------|
| ★ X5 抖动根因 | uint32 时间戳下溢 — loop 缓存 now 与阻塞后 millis() 打戳混用, 负差回绕恒判超时, PWM 5Hz 振荡 → 修复 (1行) + bypass 隔离取证法 |
| CH6 纵深防御 | 空闲间隔帧同步 (消除数据字节 0x0F 错位锁定) + ORE 丢帧 + 5帧中值/Schmitt滞回 + 非对称确认 + CH5 帧同步防抖 + 诊断计数器 |
| WFLY 校准 | raw 中位=1024 (非 992), SBUS_CENTER=1024/量程 352-1695, 满杆精确 ±500μs + c1=/c2= 原始值输出 |
| 全速化 | 手控全量程 + motor_bridge 增益 1000/600 + PWM 斜率限制移除 (用户决策, 接受欠压风险) |
| 安全加固 | /locked_track_id 跟随门控 + CI1302 语音 500ms 冷却 + 语音动作 10Hz 独立 timer + keepalive 20Hz |

### Phase 6: 稳定性 + 系统优化 (v0.9.0)

| 里程碑 | 内容 |
|--------|------|
| ★ CI1302 V6 | 语义 0x04/0x05 锁定/解除跟随者双向通信, 手势→语音确认反馈, 语音→手势等效 relay |
| ★ 锁稳定性 | RE-ID 保持窗口 1s (修复秒切 bug), 搜索半径 150→80px, 急停豁免被锁人 |
| ★ 横向 PD | 纯 P→PD (k_p=0.4, k_d=1.2), ±5cm 死区, 低通滤波 α=0.25, staleness 1→0.3s |
| ★ 后退丝滑 | Schmitt 迟滞 + 速度地板(-0.15) + EKF vx 前馈 + 20Hz 独立定时器 |
| 手势 Phase 3 | Victory(✌️) 并行锁定, 滑动窗口投票, 置信度门控, 空间 fallback, 自适应发现 |
| 系统优化 | 启动 62s→20s (禁用 8 无用服务), 磁盘 13G→9.6G, PWM 日志炸弹修复 |
| 参数调优 | angular_gain 600→450, PERSON_STALE_MAX 30→15 |

---

## 6. 关键设计决策

### 6.1 为什么用 LiDAR 测距而非纯视觉?

- **精度**: LiDAR ≈2cm vs bbox 启发式 ±30% 误差
- **鲁棒性**: 不受光照/衣服颜色/人体姿态影响
- **速度**: LiDAR 10Hz 独立测距 vs 视觉法依赖 bbox 宽度估计
- **决定性转折**: 双目深度 (StereoNet) 与检测模型无法在 X5 上并发 (BPU 单核互斥)

### 6.2 为什么胸高度安装 LiDAR?

- 物理限制: 车体结构决定传感器安装高度 ~150cm
- **结果**: 扫描到躯干连续弧面 (不是腿高度常见的两根分离柱)
- **影响**: Leg Detection 方案不适用, 改为躯干几何过滤 (弧宽+曲率)
- 优势: 胸高度不易被遮挡, 且 LiDAR 光束与相机 FOV 在水平角上对齐

### 6.3 为什么用 5 层控制层级?

- **安全**: RC CH5 油门锁 → 硬件急停 (不经过任何软件)
- **授权**: RC CH6 → 上位机接管 vs 纯 RC 直通 (避免 ROS2 崩溃失控)
- **渐进**: 语音 → 跟随 → 手势, 每层增加自主性
- **防御深度**: 任何一层失败, 上一层立即接管

### 6.4 为什么 motion_arbiter 是 /cmd_vel 唯一发布者?

- **消除冲突**: 多节点同时写 /cmd_vel 导致抖动/竞争
- **单点仲裁**: 语音命令 + LiDAR 距离 + 急停, 优先级在此层统一决策
- **可审计**: 所有运动命令经过一个节点, 易于调试

### 6.5 为什么 voting 阈值从 30 降到 15?

- **根本原因**: image_gap=2 导致手势特征采样率 60→30fps, gestureDet 置信度显著下降
- **解决**: 恢复 image_gap=1 (60fps), 配合阈值减半, 触发时间 ~15/60=0.25s
- **C++ 层已有投票**: gestureDet 内部 time_interval=0.25s, Python 层无需重复 1 秒验证

### 6.6 为什么 Adaptive Threshold > DBSCAN?

- **性能**: 自适应 O(N) vs DBSCAN O(N log N) → 55ms vs 75ms
- **精度差**: F1 92% vs 94% (差异仅 2%, 且我们有人体检测辅助)
- **简单性**: 10 行代码, 零依赖 vs scikit-learn
- **论文支撑**: Zhu Wang et al. (2021) 证明自适应 Euclidean 在 2D LiDAR 场景下接近 DBSCAN

---

## 7. 数据结构总览

### 话题速查表

| Topic | 类型 | 方向 | 频率 | 用途 |
|-------|------|------|------|------|
| `/image` | CompressedImage | codec→perception | 60fps | HDMI 渲染 |
| `/hobot_mono2d_body_detection` | PerceptionTargets | mono2d→perception,hand_lmk,body_track | 60fps | 人体+手 ROI |
| `/hobot_hand_lmk_detection` | PerceptionTargets | hand_lmk→hand_gesture | ~30fps | 手部关键点 |
| `/hobot_hand_gesture_detection` | PerceptionTargets | gesture→perception | 2-4fps | OK/Palm 手势码 |
| `/scan` | LaserScan | ydlidar→perception | 10Hz | LiDAR 点云 |
| `/follow_active` | Bool | arbiter→perception | event | 跟随模式指示 |
| `/cmd_vel_body_track` | Twist | body_track→arbiter | 30fps | 视觉跟随命令 |
| `/locked_target` | **Point** | perception→arbiter | 15fps | x=距离, y=侧向 |
| `/locked_track_id` | Int32 | perception→arbiter | event | 锁定行人 ID |
| `/emergency_stop` | Bool | perception→arbiter | 15fps | 急停标志 |
| `/system_ready` | Bool | perception→arbiter | once | 启动就绪 |
| `/cmd_vel` | Twist | arbiter→motor_bridge | ~30fps | **最终运动命令** |

### MotorCmd 帧格式 (X5 → STM32)

```
[0xAA][throttle_lo][throttle_hi][steering_lo][steering_hi][CRC8]
  6 bytes @ 115200 bps
  throttle/steering: uint16 LE, 1500us=停止, 1000-2000us 范围
  CRC8: poly=0x07, init=0x00, 覆盖 byte1-4
```

### CI1302 语音帧 (A5 FA 协议)

```
[A5][FA][00][TYPE][CMD_ID][00][CKSUM][FB]  8 bytes @ 115200
  TYPE=0x81: CI1302→X5 (识别结果)
  TYPE=0x82: X5→CI1302 (触发播报)
  CKSUM = (A5+FA+00+TYPE+CMD+00) & 0xFF
```
