# 6WD Heavy Tracked Vehicle — RDK X5 Autonomous Follower

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![RDK](https://img.shields.io/badge/RDK-X5-brightgreen)](https://developer.d-robotics.cc/rdk_doc/RDK)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)](./LICENSE)

> 以地平线 RDK X5 为大脑、亚博 STM32 V3.0 扩展板为小脑、双目深度 + YOLO 人体检测为感知链的**六轮重载履带自主跟随机器人**。

---

## 📖 概述

本项目将一套原基于 **ESP32 + OpenMV** 的 6WD 履带跟随车迁移至 **RDK X5 + STM32 ROS 主控板** 架构，实现：

- **YOLO 人体检测 + 双目深度测距** — 方案 B：左目 YOLO 检测人体框，双目深度图采样框内点估算距离
- **distScore 跟随算法** — 移植自 ESP32 FollowLogic，双向比例控制（前进/后退/转向）
- **分层安全架构** — 专业遥控器 SBUS 直连 STM32 最高优先级，X5 指令次之，命令超时自动刹停
- **多传感器融合** — 激光雷达避障、AI 语音交互、后视辅助
- **全 ROS2 Humble + TROS 生态** — 一键 launch 全系统启动

---

## 🏗️ 硬件架构

```
                         ┌─────────────────────┐
                         │   专业遥控器 (SBUS)    │
                         │     最高优先级        │
                         └──────────┬──────────┘
                                    │ SBUS (UART)
                                    ▼
┌──────────────┐  MIPI CSI   ┌─────────────┐  UART0  ┌──────────────────┐
│  GS130W 双目  │◄──────────►│   RDK X5    │◄───────►│ STM32 ROS 主控板  │
│  左目/右目    │            │  (L2 决策层) │MotorCmd │   (V3.0 · L1)    │
└──────────────┘            │             │         │ 坦克混控·IMU·SBUS│
                            │  YOLO 检测   │         └────────┬─────────┘
┌──────────────┐  UART1     │  深度融合    │                  │ PWM×2
│ T-mini Plus  │◄──────────►│  跟随决策    │         ┌────────▼─────────┐
│ 激光雷达 12m  │            │  雷达融合    │         │ ZTW Seal G2 ×2   │
└──────────────┘            │  语音控制    │         │ 双路无刷电调       │
                            │  安全看门狗   │         └────────┬─────────┘
┌──────────────┐  UART2     │             │                  │ 三相无刷
│  AI 语音模块  │◄──────────►│             │         ┌────────▼─────────┐
└──────────────┘            └──────┬──────┘         │ 电机L · 电机R     │
                                   │ VIS帧(UART3)   └──────────────────┘
┌──────────────┐                   │
│ OpenMV N6    │◄──────────────────┘
│ 后视辅助      │  @4800bps
└──────────────┘
```

### 安全优先级

```
SBUS 遥控器 (STM32 直连)  ▸  最高优先，X5 指令可被覆盖
RDK X5 自主指令 (UART)    ▸  遥控器断开时生效
超时关断 (60s 无新命令)    ▸  硬件级安全兜底
```

---

## 📋 物料清单 (BOM)

| 组件 | 型号 / 规格 | 用途 |
|------|-----------|------|
| 主控计算 | **RDK X5** (X5U SoC, 10 TOPS BPU, 4GB) | 视觉推理、决策、ROS2 主节点 |
| 底盘控制 | **亚博 STM32 ROS 扩展板 V3.0** | 电机控制、IMU、SBUS 接收 |
| 双目相机 | RDK GS130W (或 132GS) MIPI 双目 | 人体检测 + 深度测距 |
| 激光雷达 | 亚博 T-mini Plus (12m) | 避障、环境感知 |
| 语音模块 | 亚博 AI 语音交互模块 | 语音指令控制 |
| 后视相机 | OpenMV Cam N6 | 后视人体检测 |
| 无刷电调 | ZTW Seal G2 ×2 | 双路三相无刷电机驱动 |
| 动力电机 | 三相无刷 ×2 | 履带驱动 |
| 电源 | 48V 89Ah 锂电池 | 全车供电 |
| 遥控器 | 专业遥控器 + SBUS 接收机 | 手动操控、安全覆盖 |

---

## 🧠 软件架构

```
                         ROS2 Humble + TROS
┌────────────────────────────────────────────────────────────┐

---

## 📁 目录结构

```
tracked-vehicle-rdk/
├── README.md                         # 项目总览（本文件）
├── LICENSE                           # MIT 开源协议
├── .gitignore                        # Git 忽略规则
├── CHANGELOG.md                      # 版本更新日志
│
├── docs/                             # 📝 设计文档
│   ├── architecture.md               #   系统架构详细设计
│   ├── hardware-setup.md             #   硬件连线与接口对表
│   ├── protocol-spec.md              #   MotorCmd / VIS / SBUS 协议定义
│   ├── migration-plan.md             #   ESP32→X5 分阶段迁移计划
│   └── safety-design.md              #   安全机制设计文档
│
├── config/                           # ⚙️ YAML 参数文件
│   ├── stereo_calib.yaml             #   双目标定参数
│   ├── yolo_detection.yaml           #   YOLO 检测配置
│   ├── motor_config.yaml             #   电机 PWM 参数
│   ├── follow_params.yaml            #   跟随算法参数
│   └── lidar_params.yaml             #   激光雷达配置
│
├── launch/                           # 🚀 ROS2 launch 文件
│   ├── stereo_vision.launch.py       #   双目采集 + 深度图
│   ├── person_detection.launch.py    #   YOLO 人体检测
│   ├── person_follow.launch.py       #   检测 + 深度 + 跟随
│   ├── motor_bridge.launch.py        #   X5↔STM32 串口桥接
│   ├── lidar_node.launch.py          #   激光雷达驱动
│   ├── voice_control.launch.py       #   语音模块接口
│   ├── rear_view.launch.py           #   后视 VIS 帧解析
│   └── full_system.launch.py         #   一键全系统启动
│
├── src/tracked_vehicle/              # 🐍 ROS2 包
│   ├── setup.py                      #   colcon 构建
│   ├── package.xml                   #   依赖声明
│   ├── tracked_vehicle/              #   核心 Python 模块
│   │   ├── __init__.py
│   │   ├── motor_controller.py       #   MotorCmd 协议生成与串口收发
│   │   ├── follow_logic.py           #   distScore 跟随算法（移植自 ESP32）
│   │   ├── person_tracker.py         #   YOLO 检测框 + 双目深度采点
│   │   ├── lidar_fusion.py           #   激光雷达避障融合
│   │   ├── rear_view_parser.py       #   OpenMV VIS 帧解析
│   │   ├── voice_commander.py        #   AI 语音指令解析
│   │   ├── sbus_monitor.py           #   SBUS 遥控器状态监听
│   │   └── safety_watchdog.py        #   60s 命令超时安全看门狗
│   └── scripts/                      # 🔧 工具脚本
│       ├── motor_calibrate.py        #   电调校准
│       ├── stereo_calib_capture.py   #   双目标定图像采集
│       ├── protocol_sniff.py         #   串口抓包调试
│       └── system_check.py           #   一键硬件体检
│
├── models/                           # 🧠 BPU 模型文件 (.bin)
│   ├── person_yolov5s_x5.bin         #   YOLOv5s 人体检测 (Bayes)
│   └── stereonet_depth_x5.bin        #   双目深度模型
│
├── stm32_firmware/                   # 🔩 STM32 扩展板固件
│   ├── platformio.ini                #   PlatformIO 构建配置
│   └── src/
│       ├── main.cpp                  #   主程序（坦克混控·SBUS·MotorCmd·PWM）
│       └── config.h                  #   引脚定义与协议常量
│
├── openmv_rear/                      # 👁️ 后视辅助
│   └── main.py                       #   后视人体检测 + VIS 帧发送
│
└── tests/                            # 🧪 单元测试
    ├── test_motor_protocol.py        #   MotorCmd 编解码
    ├── test_follow_logic.py          #   跟随算法边界条件
    ├── test_vis_parser.py            #   VIS 帧解析
    └── test_safety_watchdog.py       #   安全看门狗
```

---

## 🔌 通信协议

### MotorCmd — X5 → STM32 运动指令

| Byte | 字段 | 说明 |
|------|------|------|
| 0 | `0xAA` | 帧头 |
| 1 | `throttle_lo` | 油门低字节 |
| 2 | `throttle_hi` | 油门高字节 |
| 3 | `steering_lo` | 转向低字节 |
| 4 | `steering_hi` | 转向高字节 |
| 5 | `CRC8` | CRC-8-MAXIM 校验 |

- 波特率：**115200 bps**
- 发送间隔：**50ms**
- 停止值：`throttle=0, steering=0`

### PWM 输出 — STM32 → 电调

| 参数 | 值 |
|------|-----|
| 频率 | 50Hz (周期 20000μs) |
| 中位 | 1275μs |
| 最小 | 650μs |
| 最大 | 1900μs |
| 左电机 | TIM3 CH3 — PB0 |
| 右电机 | TIM3 CH4 — PB1 |

### VIS 帧 — OpenMV → X5

```
"VIS:cx,cy,w,h,feetY,conf,PERSON,distScore,tofDist*CRC8\\r\\n"
```

- 波特率：**4800 bps**
- 校验：XOR checksum over payload (before `*`)

---

## 🛡️ 安全机制

| 层级 | 机制 | 描述 |
|------|------|------|
| L1 | **SBUS 遥控器优先** | 遥控器直连 STM32，指令硬件级优先于 X5 |
| L2 | **命令超时刹停** | X5 超过 60s 无新 MotorCmd → STM32 自动切中位 |
| L3 | **X5 安全看门狗** | ROS2 节点心跳监控，异常时主动发停止指令 |
| L4 | **电调物理保护** | ZTW Seal G2 内置过流/过热/堵转保护 |
| L5 | **视觉丢帧暂留** | 人体检测丢失 ≤5 帧维持上一指令，避免急刹 |
| L6 | **激光雷达紧急制动** | 检测到障碍物 < 安全距离 → 强制减速/停止 |

---

## 🚀 快速开始

### 前置条件

```bash
# 1. RDK X5 已刷 RDK OS 3.x (Ubuntu 22.04 + ROS2 Humble)
cat /etc/version

# 2. TROS 环境可用
source /opt/tros/humble/setup.bash
ros2 pkg list | grep hobot_dnn

# 3. 依赖安装
sudo apt install -y python3-pip i2c-tools gpiod
pip3 install pyserial pyyaml
```

### 克隆与构建

```bash
cd ~/Desktop
git clone <your-repo-url> tracked-vehicle-rdk
cd tracked-vehicle-rdk

# 构建 ROS2 包
source /opt/tros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 烧录 STM32 固件

```bash
cd stm32_firmware
pio run --target upload
```

### 一键启动

```bash
source /opt/tros/humble/setup.bash
source install/setup.bash
ros2 launch tracked_vehicle full_system.launch.py
```

### 子系统独立启动

```bash
# 仅双目 + 深度
ros2 launch tracked_vehicle stereo_vision.launch.py

# 仅人体跟随
ros2 launch tracked_vehicle person_follow.launch.py

# 仅电机控制
ros2 launch tracked_vehicle motor_bridge.launch.py
```

### 硬件体检

```bash
python3 src/tracked_vehicle/scripts/system_check.py
```

---

## 📚 文档索引

| 文档 | 内容 |
|------|------|
| [docs/architecture.md](./docs/architecture.md) | 系统架构详细设计 |
| [docs/hardware-setup.md](./docs/hardware-setup.md) | 硬件连线与接口对表 |
| [docs/protocol-spec.md](./docs/protocol-spec.md) | 通信协议完整定义 |
| [docs/migration-plan.md](./docs/migration-plan.md) | ESP32→X5 分阶段迁移计划 |
| [docs/safety-design.md](./docs/safety-design.md) | 安全机制设计 |

---

## 🧪 测试

```bash
cd tests
python3 -m pytest test_motor_protocol.py
python3 -m pytest test_follow_logic.py
python3 -m pytest test_vis_parser.py
```

---

## 🛤️ 路线图

- [x] M1：硬件选型与采购
- [x] M2：目录结构与项目骨架
- [ ] M3：STM32 固件适配（SBUS + MotorCmd 双源）
- [ ] M4：RDK X5 视觉验证（双目 + YOLO + 深度）
- [ ] M5：跟随算法移植（distScore → MotorCmd）
- [ ] M6：传感器逐一接入（激光雷达、语音、后视）
- [ ] M7：全系统联调与安全验收
- [ ] M8：场地实车测试

---

## 🤝 贡献指南

1. Fork 本仓库
2. 创建特性分支：`git checkout -b feat/xxx`
3. 遵循现有代码风格（Python: PEP8, C++: Google Style）
4. 提交前运行测试：`colcon test`
5. 提交 Pull Request 并描述变更

---

## 📄 许可证

本项目基于 **MIT License** 开源。详见 [LICENSE](./LICENSE)。

---

## 🙏 致谢

- [地平线 D-Robotics](https://developer.d-robotics.cc/) — RDK X5 硬件与 TROS 生态
- [亚博智能 Yahboom](https://www.yahboom.com/) — STM32 ROS 扩展板与传感器套装
- 原 ESP32+OpenMV 项目 (6wd-follower-esp32-openmv) — 跟随算法与协议设计基础

---

<p align="center">
  <b>Built with ❤️ on RDK X5 · ROS2 Humble · TROS</b>
</p>
*** Add File: /home/sunrise/Desktop/tracked-vehicle-rdk/.gitignore
# Python
__pycache__/
*.py[cod]
*.so
*.egg-info/
dist/
build/
*.egg

# ROS2
install/
log/

# STM32 / PlatformIO
.pio/
.vscode/
*.elf
*.hex

# Models (large files)
*.bin
*.tflite
*.onnx
*.hbm

# Environment
.env
.env.local
venv/
.venv/

# IDE
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db

# Debug
*.log
**/logs/

# Calibration artifacts
*.calib_cache

*** Add File: /home/sunrise/Desktop/tracked-vehicle-rdk/LICENSE
MIT License

Copyright (c) 2026 Tracked Vehicle RDK Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*** Add File: /home/sunrise/Desktop/tracked-vehicle-rdk/CHANGELOG.md
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.1.0] - 2026-06-15

### Added
- Project skeleton: directory structure, README, LICENSE, .gitignore
- Architecture design: dual-layer (RDK X5 L2 + STM32 L1) with SBUS safety override
- Documentation placeholders: architecture, hardware setup, protocol spec, migration plan, safety design
- ROS2 package scaffold: `src/tracked_vehicle/` with module stubs

*** End Patch