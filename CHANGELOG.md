# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

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
