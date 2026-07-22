# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.9.0] - 2026-07-22

### Added

- **CI1302 V6 固件适配** — 语义 ID 0x04 (锁定跟随者) / 0x05 (解除跟随者) 双向通信
- **手势语音反馈** — FOLLOWING 模式下手势锁/解锁自动触发 CI1302 播报确认
- **语音→手势 relay** — 用户说"锁定跟随者"/"解除跟随者" → `/voice_gesture_cmd` → perception_node 等效手势操作
- **Victory (✌️) 并行锁定** — 滑动窗口投票 + 多码并行 `lock_codes=[11,2]` + 置信度门控 + 空间 fallback
- **自适应手势发现** — 新出现手势码自动打印到日志, HDMI 实时进度条
- **EKF 速度前馈** — `/locked_target.z` 发布 EKF vx, motion_arbiter 用于预判后退

### Changed

- **横向控制: P→PD** — `k_p=0.4, k_d=1.2`, ±5cm 死区, 低通滤波 α=0.25
- **后退逻辑重写** — Schmitt 迟滞 (进 <0.85m / 出 >1.0m), 速度地板 -0.15 m/s, 0.5m 分段渐变
- **锁稳定性修复** — RE-ID 保持窗口 1s (之前第 1 帧即切换), 搜索半径 150→80px
- **20Hz 独立跟随定时器** — 近距相机遮挡时不依赖 body_track 消息
- **急停豁免被锁人** — 被锁目标角度 ±15° 内障碍物不再触发紧急停止
- **参数调优** — `angular_gain 600→450`, staleness `1.0→0.3s`, `PERSON_STALE_MAX 30→15`

### Fixed

- **PWM 日志炸弹** — motor_bridge 周期状态 WARN→DEBUG, 轮询 0.5→2s
- **日志清理** — `/root/.ros/log/` (1.9GB), `/var/log/syslog` (632MB), journal vacuum

### Optimized

- **启动时间 62s→20s** — 禁用 8 个无用 systemd 服务
- **CPU** — 移除重复 `fusion.update()` 调用
- **磁盘 13G→9.6G** — apt 缓存 + autoremove

## [0.8.2] - 2026-07-21

### Fixed

- **X5 模式原地剧烈抖动 (<5cm)** — uint32 时间戳下溢根因修复 (潜伏自 v0.8.1)
- **WFLY 遥控器校准** — SBUS 通道映射实测修正
- **CH6 纵深防御** — 5 帧中值滤波 + Schmitt 滞回 + 非对称稳定确认

### Changed

- **全速化** — 移除斜率限制, 欠压风险靠供电侧解决
- 急停 angular 泄漏修复
- 死代码全面清理

## [0.8.1] - 2026-07-20

### Changed

- **STM32 X5 模式超时语义** — 自动驾驶模式 (CH6=HIGH) 下 X5 指令超时不再自动锁定: 仅输出中位停车待命, 指令恢复即继续. 锁定只能由遥控器 CH5 手动打到锁定位置 (或 SBUS 信号丢失) 触发. 手控模式超时锁定逻辑不变
- **渲染降频** — perception_node 30→15Hz, `cv2.waitKey` 30→1ms
- **JPEG 按需解码** — img_cb 不再对每帧解码 (60fps), 仅在 render (15Hz) 时解码最新帧, JPEG 解码量降为 1/4
- **body_tracking 日志级别** — launch 中固定为 error (该节点 WARN 级日志 ~60Hz rotate/cancel churn, 曾刷出 2.3GB journal 并淹没崩溃现场证据)

### Added

- **PWM 斜率软启动 (STM32)** — 加速方向限速 1200μs/s (0→满行程 ~0.4s), 抑制电机启动/换向浪涌电流, 防止母线电压跌落导致 X5 欠压硬重启; 减速/回中/急停不限速, 立即生效
- **STM32 调试输出转发** — motor_bridge 读取 STM32 在同一 USART1 上的调试打印, 关键事件 ([SAFE]/[SBUS]/[MODE]/ARM/DISARM/启动 banner) 转发到 ROS 日志; STM32 意外复位 (IWDG/掉电) 从此在 journal 中留下 banner 痕迹
- **语音动作指令重发** — motion_arbiter 在 3s 动作窗口内以 5Hz 持续重发语音运动命令, 保持 STM32 指令流连续 (修复单帧指令 2s 后被 STM32 判超时停车、实际动作时长不足 3s 的问题)

### Fixed

- pyserial `OSError` (Errno 5, USB 串口抖动) 未被捕获可致节点崩溃的隐患 — motion_arbiter / motor_bridge 统一捕获 `SerialException + OSError`

## [0.7.0] - 2026-07-13

### Changed

- **节点架构重构** — 感知/仲裁/执行三层分离:
  - `display_node` 合并入 `perception_node` (LiDAR融合 + 手势锁定 + HDMI屏显 + 系统监控, 单一权威源)
  - `voice_bridge` 重构为 `motion_arbiter` (/cmd_vel 唯一发布者, 状态机 VOICE_MANUAL↔FOLLOWING)
  - `cmd_vel_bridge` 重命名为 `motor_bridge` (语义更精确, 职责仅为串口桥接)
- **LiDAR 距离覆写** — motion_arbiter 在 FOLLOWING 模式下用 LiDAR EKF 融合距离覆写 linear.x, 保留 bbox angular.z
- **渲染降频** — 60 → 30Hz (perception_node 屏显), 轮询 10 → 5Hz (motion_arbiter), 降低 CPU/BPU 占用
- **跟踪丢失阈值** — `track_serial_lost_num_thr` 300 → 150 帧, 更快切 STOP
- **mono2d image_gap** — 引入 image_gap=2, 检测帧率 60 → 30FPS
- **手势识别优化** — image_gap 回退 (恢复 60FPS), 投票阈值 30→15, 删除空间启发式 fallback, 纯属性码 OK/Palm 通道
- **LiDAR 融合管线重构**:
  - 自适应聚类阈值 (近 0.10m ↔ 远 0.40m, 参考 Zhu Wang et al. 2021)
  - 躯干几何过滤 (弧宽 15-70cm + 曲率 <0.97, 排除墙壁/柱子)
  - 贪心匹配 → 匈牙利全局匹配 (scipy linear_sum_assignment)
  - EKF Q 矩阵 ×dt 缩放 (消除 60Hz predict 6x 噪声累积)
  - Cartesian 质心替代径向距离均值
- **运动控制增强**:
  - 速度映射连续化 (0.7m 边界 15cm 过渡区, 消除震颤)
  - vel_fast 0.3→0.8 m/s, vel_back -0.2→-0.3 m/s, dist_far 2.5→3.0m
  - LiDAR 侧向偏移驱动转向 (k=0.5 rad/s/m)
- **/locked_target 升级** — Float32 → Point (x=距离, y=侧向偏移)
- **Camera FOV 修正** — 70°→72° (SC132GS 校准文件 fx=656.76 精确推算)

### Added

- **障碍物紧急停止** — /emergency_stop topic, 前方 ±15°/0.5m 触发急停
- **/camera_info 订阅** — 自动获取真实内参 FOV (fallback 72°)
- **camera_frame 静态 TF** — base_link → camera_frame
- **python3-scipy 依赖** — 匈牙利匹配

### Fixed

- perception_node `follow_on` 变量未定义 (NameError)
- 障碍物 ID 不持久 (序号 → 角度匹配, 扫描间稳定)
- 障碍物更新绕过 MAX_DIST_JUMP 门控 (已加保护)
- dist_mean 径向均值 → Cartesian 质心 (宽角度聚类几何误差)
- LiDAR 点数注释修正 (900→430, 实测 fixed_resolution 输出)

## [0.6.0] - 2026-07-03

### Changed

- **voice_bridge 协议升级** — CI1302 固件 V00681 → V01843, AA 55 → A5 FA (8B 帧, 累加和校验, CMD 映射重新编号).
  协议详情见 `docs/protocol-spec.md#4-ci1302-a5-fa` 及 `ci1302_firmware/sfw*/readme.txt`.
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
  - `docs/reference/voice_module/`: Speech_Lib + RDKX5/UART/ROS1/ROS2 参考 (已精简, 协议统一至 docs/protocol-spec.md)

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
- **stm32flash 烧录经验** — bootloader 时序 + CH340N USB 异常恢复 + udev 端口固定

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
