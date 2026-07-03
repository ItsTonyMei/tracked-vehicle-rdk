# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.6.0] - 2026-07-03

### Changed

- **voice_bridge 协议升级** — CI1302 固件 V00681 → V01843, AA 55 → A5 FA
  - 帧格式: `A5 FA 00 81 [CMD] 00 [CKSUM] FB` (8 bytes), 带累加和校验
  - CMD 映射重新编号: 停止 0x06, 前进 0x07, 后退 0x08, 左转 0x09, 右转 0x0A, 左旋 0x0B, 右旋 0x0C, 跟随开 0x0D, 跟随关 0x0E
  - 唤醒词 "你好瓦力" (CMD 0x01) 由 DNN 级独立模型门控 (`USE_SEPARATE_WAKEUP_EN=1`)
- **欢迎语自动播报** — display_node 启动完成 (30s) 后通过 `/system_ready` topic 通知 voice_bridge 触发欢迎语, 与屏幕 "ALL SYSTEMS GO" 精确同步
- **协议表重建** — 移除垃圾分类/机器人演示/颜色识别等 90+ 条无用命令词, 精简为 14 条履带车专用协议

### Added

- **`/system_ready` topic** — display_node 启动完成后发布, voice_bridge 订阅后触发欢迎语 (替代盲等定时器)
- `ci1302_firmware/sfw20260703134807158195173/` — V01843 v2 固件 (内部RC + 关闭波特率校准)
- `ci1302_firmware/命令词播报词协议列表V3_履带车.xlsx` — 新版协议表 (14 条, 含唤醒词配置)

### Removed

- 旧版 `命令词播报词协议列表V3_中文_瓦力.xlsx` (110 条, 含大量演示命令)

### Fixed

- CI1302 开机误播垃圾分类 (0x67 命令冲突, 已在固件层面彻底解决)
- 语音命令无唤醒词直接生效 (DNN 分离模型 + 唤醒词 gating)
- CI1302 串口通信乱码 — 时钟源配置错误 (外部晶振 → 内部RC) + 关闭波特率校准 (踩坑 #26)

## [0.5.2] - 2026-06-25

### Added

- **目标锁定状态机 v2** — 手势-人体空间匹配 + HOLDING 暂留 + 无人重置
  - `_match_gesture_to_person()`: 手势 ROI 中心点与 body ROI 空间匹配, 锁定做 OK 的人
  - HOLDING 状态: 被锁者消失 <5s 维持锁定; 空间重识别 (RE-ID) 自动匹配新 track_id
  - `_empty_since` 计时器: 画面无人 >10s 自动清除锁定
  - `det_cb` 基于 body ROI 检测无人 (而非 person type, 后者永远为 True)
- **X5 系统状态栏** (HDMI 左上角)
  - CPU%/BPU%/MEM%/TEMP 实时显示 (1Hz 刷新)
  - 彩色状态指示灯: 绿(<80%) / 黄(<95%) / 红(>=95%)
  - 节点计数: `pgrep` 后台扫描, 非阻塞; 启动自检消息行
  - FPS 健康指示灯: 绿>30 / 黄10-30 / 红<10
- **AI 语音模块集成** — `voice_bridge.py` CI1302 → `/cmd_vel` ROS2 节点
  - 20Hz 轮询 `/dev/ttyUSB1`, 解析 `AA 55 [STATUS] [ID] FB` 帧
  - 8 条出厂语音指令映射 (前进/后退/左右转/旋转/停止), 动作 3s 自动停止
  - `docs/reference/voice_module/`: Speech_Lib + RDKX5/UART/ROS1/ROS2 完整参考

### Changed

- **屏显渲染优化**
  - 中心十字: 白色加粗 (255,255,255), 线宽 2, 长度 25px
  - bbox→中心连线: 跟随 bbox 颜色 (锁定=红/未锁=绿/HOLDING=橙), 线宽 2
  - 闪框颜色语义化: 锁定=红色, 解锁=绿色
  - 字体放大: FONT_SCALE 0.5→0.7, FONT_THICK 1→2, LABEL_H 24→32
  - DETECT 行只显示 body ID (去重 head/face/hand)
- **手势回调修复**: 恢复 early return 防止后续 `gesture=0` 清零投票
- **手势匹配修复**: `_match_gesture_to_person` 只检查 `hand` 类型 ROI
- **CPU 占用率修复**: `_prev_cpu_jiffies` 变量名不一致导致始终为 0%
- **服务单元优化**: 添加 `KillMode=mixed` + `TimeoutStopSec=30` 修复重启卡死

### Removed

- **Web 可视化**: 禁用 websocket 节点 (~14% CPU + 69MB RAM)
- **nginx**: 停止并禁用 Web 服务器
- **桌面组件**: 清除 xubuntu-desktop/gnome-shell/lightdm/xfce/snapd (~1.3GB 磁盘)
- **系统服务**: 禁用 tftpd-hpa/accounts-daemon/rpcbind/udisks2/hobot-suspend-button 等 8 个
- **EXPECTED_NODES**: 11→10 (websocket 移除); 10→11 (voice_bridge 新增)

### Verified

- 板端 X5 重启后内存 used: 531MB→250MB (idle) / 527MB→490MB (running)
- 系统服务: 25→17 个
- 10 节点全链路稳定 (不含语音模块时为 9+1 待启动)
- STM32 IWDG/SBUS/USART/蜂鸣 修复验证通过
- CI1302 语音模块协议确认: 115200 8N1, `AA 55 00 [ID] FB`

## [0.5.1] - 2026-06-24

### Fixed (P0 — Critical)

- **CRC-8 位溢出** — `cmd_vel_bridge.py` 移位循环加 `& 0xFF` 掩码，与 STM32 `uint8_t` 行为一致
- **IWDG 硬件看门狗** — STM32 启用独立看门狗 4s 超时，`loop()` 末尾喂狗
- **SBUS 帧丢失盲区** — `lost_frame` 不再拒绝整帧，改为设置 `failsafe=true` 阻断手动控制
- **USART 寄存器违规** — `sbusInit()` 先禁用 USART (UE=0) 再修改 M/PCE 位
- **阻塞蜂鸣** — 删除 `beep()`/`beepArm()`/`beepDisarm()`，统一为非阻塞蜂鸣状态机

### Fixed (P1 — High)

- **QoS** — `display_node` 相机帧/检测订阅改用 `BEST_EFFORT` (depth=1/5)
- **过期检测保护** — `_on_ok` 拒绝超过 500ms 的过期检测数据
- **看门狗空转** — `cmd_vel_bridge` 检查间隔 0.5s→`min(5s, timeout/10)`
- **串口自动重连** — 写失败时尝试关闭重开串口
- **CMD_TIMEOUT** — STM32 侧从 60s 降至 2s (X5 指令间隔约 30ms)
- **代码规范** — 订阅移入 `__init__()`，`zip_safe=False`，`log_level` 可配置
- **文档去重** — `lessons-learned.md` 与 `main.cpp` 交叉引用

### Fixed (P2 — Medium)

- **坦克混控去重** — 提取 `computeMix()` 消除两处重复计算
- **MPU9250 PLL** — 时钟源从内部振荡器改为 PLL+GyroX (0x01)，提升陀螺仪精度
- **手势多目标** — `gesture_cb` 遍历所有目标累积投票
- **PEP8** — `display_node.py` 修复分号语句
- **steering_invert** — `cmd_vel_bridge` 添加可配置转向方向参数
- **移除死代码** — `g_sbus.armedPrev` 未使用字段

### Added

- **flash_stm32.sh** — STM32 一键烧录脚本 (stm32flash + bootloader 轮询, 30s 窗口)
- **udev 端口固定** — `/etc/udev/rules.d/99-tracked-vehicle.rules` CH340N→`/dev/stm32_board`
- **烧录经验文档** — `memory/stm32-flash-experience.md` (6 条踩坑 + 恢复流程)

## [0.5.0] - 2026-06-22

### Added
- **手势唤醒跟随** — OK 锁定 / Palm 解除
  - `display_node` 订阅 `/hobot_hand_gesture_detection`, 投票防抖 30帧
  - OK(11)→锁定画面最大面积人体, 红框粗线标识; Palm(5)→解除
  - 3s冷却 + 未锁定忽略Palm, 防误触发
- **屏显系统** — HDMI 本地显示全链路
  - OpenCV 全屏渲染 (1024×600), 缩放+居中裁切
  - 人体框(绿/红) + track_id + 距离标签 + 偏移线 + 中心十字
  - 裸 Xorg `-nocursor` 隐藏光标
- **systemd 开机自启** — `tracked-vehicle-display.service`
  - BPU 就绪等待 + Xorg 启动 + ROS2 全系统 launch
  - lightdm 已禁用

### Changed
- 精简为官方 body_tracking (手势版) 10节点: shm→cam→jpeg→mono2d→lmk→gesture→track→bridge→display→web
- mipi_cam 显式传 rotation=90 (GS130W SC132GS 竖屏传感器适配)
- 串口名固定为 `/dev/stm32_board` (udev规则)

### Fixed
- README 代码块完整性: 修复架构图未关闭 ``` 导致目录树渲染错乱
- 屏显方向: 先画框再缩放, 保持坐标系一致
- 相机旋转: 手动启动组件链确保 rotation=90 正确传递

### Verified
- 10节点全链路稳定运行: 相机→检测→手势→跟随→桥接→屏显→Web
- STM32 安全门控: X5指令需 autoMode && g_motorArmed
- 方向映射: angular.z 符号修正, 匹配坦克混控

## [0.4.0] - 2026-06-22

### Added
- **distScore 人体跟随节点** — `person_tracker.py`
  - bbox 宽度反比距离估算 (人体肩宽 ~0.5m 恒定)
  - 比例控制: dist_error → throttle, center_error → steering
  - 直接输出 MotorCmd (绕过 /cmd_vel)
  - 目标丢失保持 (max_lost_frames) + 60s 安全超时
- `launch/person_follow.launch.py` — 检测 + distScore 跟随一键启动

## [0.3.0] - 2026-06-19

### Added
- **人体跟随功能块** — mono2d_body_detection + body_tracking + cmd_vel_bridge
  - 人体检测+多目标跟踪 @ 60 FPS, 10ms 推理 (BPU 加速)
  - 验证通过: GS130W 单通道 960×544, rotation=90, sc132gs calibration
  - `src/tracked_vehicle/tracked_vehicle/cmd_vel_bridge.py` — cmd_vel → MotorCmd (0xAA) 串口桥接
  - `launch/full_system_tracking.launch.py` — 人体跟随 + 桥接一键启动
  - `launch/motor_bridge.launch.py` — 独立串口桥接启动
  - ROS2 包正式化: `setup.py`, `setup.cfg`, `package.xml`, `resource/`
- **双目视觉功能块** — GS130W + StereoNet V2.4_int8 深度估计 pipeline ✅
  - `launch/stereo_vision.launch.py` — 一键启动双目采集 + 深度推理 + Web 可视化
  - `config/stereo_calib.yaml` — 双目标定参数 (内参、基线、渲染配置)
  - `src/tracked_vehicle/scripts/camera_info_repub.py` — camera_info 尺寸缩放工具
  - `docs/stereo-vision-verification.md` — 完整验证文档 (参数、性能、踩坑记录)

### Changed
- 仓库整理: 调试脚本移至 `docs/reference/`, PDF 移至 `docs/`, 空目录加 `.gitkeep`
- README 目录结构标注 ✅/⬜ 实现状态; 路线图 M4 标记完成

### Fixed
- README PWM 参数与 config.h 同步 (1500μs 中位 / 1000-2000μs 范围)
- 注释统一: 1500μs 为标准舵机 PWM 通用规范, 1275μs 为 C06B 非标误用

### Verified
- GS130W 双目在 RDK X5 上稳定运行: 640×352@30fps → StereoNet V2.4_int8 → 21.3fps 深度图
- Body Tracking 在 RDK X5 + GS130W 上稳定运行: 960×544@60fps, 人体检测 10ms 推理
- `/cmd_vel` (linear=0.2, angular=0.4) → MotorCmd 桥接逻辑验证通过
- 关键参数: `mipi_rotation=90.0` (SC132GS 竖屏 sensor 必须), `mipi_channel=0,2` (左右目顺序)

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
