# 踩坑经验记录

## RDK X5 平台

### 1. GitHub README 渲染 — 代码块必须成对关闭
- **症状**: 目录树结构在 GitHub 上完全错乱, 树形字符变成 markdown 语法
- **根因**: 前方软件架构图的 ```` ``` ```` 代码块缺少关闭标记, 总数变为奇数
- **修复**: `grep -c '```' README.md` 验证偶数, 补上缺失的关闭标记
- **教训**: README 修改后必须验证 ``` 成对; 报目录树渲染异常时先查奇偶性

### 2. MIPI 相机 rotation 必须显式传递
- **症状**: GS130W 画面方向不对
- **根因**: SC132GS 是竖屏 sensor (1088×1280), 需要 `mipi_rotation=90`
- **教训**: 使用官方 launch 时注意它不会自动传 rotation, 需要手动分离相机启动

### 3. hobot_hdmi DRM 直显 vs X11
- **hobot_hdmi** 使用 DRM/KMS 直接渲染, 不依赖 X11, 性能更好
- 但需要正确的 DRM connector/crtc/plane ID 配置 (vs-drm 驱动)
- 与 Xorg 互斥 (DRM master 独占)
- 我们的 1024×600 屏幕没有标准 HDMI mode 960×544, 报 `Mode not found`
- **结论**: 用 X11 + OpenCV 更简单可靠

### 4. X11 光标隐藏
- `xsetroot -cursor` / Python ctypes 透明光标 → 无效
- **正确方案**: Xorg 启动参数 `-nocursor` (X Server 层面全局禁用)
- 参考: https://forum.d-robotics.cc/t/topic/28158

## 显示系统

### 5. OpenCV 窗口大小 — 裸 X 下 WINDOW_FULLSCREEN 不生效
- **症状**: 窗口只有 400×300, 不填充屏幕
- **根因**: 没有窗口管理器时, OpenCV QT 后端的 fullscreen 标志不生效
- **修复**: 显式 `cv2.resizeWindow(1024, 600)` + `cv2.moveWindow(0, 0)`

### 6. 检测框坐标系 — 先画框再缩放
- **症状**: 检测框位置与实际人体不对应
- **根因**: 框坐标在原图空间, 但缩放+crop 后才绘制框
- **修复**: 在原始 960×544 图上先画所有标注, 再整体缩放填充屏幕

### 7. CompressedImage vs Image
- **症状**: 屏显画面一直灰色, 无相机内容
- **根因**: `/image` topic 发布的是 `sensor_msgs/CompressedImage` (JPEG), 但代码订阅了 `sensor_msgs/Image`
- **修复**: 订阅 `CompressedImage`, 用 `cv2.imdecode` 解码

## ROS2 系统

### 8. systemd 服务 — WorkingDirectory 决定模型路径
- **症状**: `config/multitask_body_head_face_hand_kps_960x544.hbm not exist`
- **根因**: mono2d 用相对路径加载模型, systemd 的默认 CWD 不是 `/root`
- **修复**: `WorkingDirectory=/root` 或 `cd /root` 在脚本开头

### 9. launch 文件必须安装到 share 目录
- **症状**: `file 'xxx.launch.py' was not found in the share directory`
- **根因**: `setup.py` 的 data_files 没包含 launch 文件
- **修复**: 用 glob 收集 launch 文件并在 data_files 中指定

### 10. model_type 参数决定是否加载成功
- `model_type: 0` → 正常加载, 推理 @ 60FPS
- `model_type: 1` → 静默崩溃 (exit code -11)
- 直接启动 Node 时需显式传递, 官方 launch 中默认是 0

## STM32 固件

> **注意**：STM32 相关的踩坑细节同时在 `stm32_firmware/src/main.cpp` 文件头注释中维护。
> 以下为要点摘要；寄存器级实现细节和最新修复记录以源代码注释为准。

### 11. PWM 中位 — 1500μs 是标准, 1275μs 是 C06B 非标
- ZTW Seal G2 标准舵机 PWM: 中位=1500μs, 范围=1000-2000μs
- 原 C06B 项目批次误用 1275μs (非标值)
- 注释和文档统一: 1500μs 是通用规范, 非批次差异

### 12. MotorCmd 方向映射
- `angular.z > 0` (ROS 左转) → `steering < 1500` → 坦克混控: 左电机慢, 右电机快 → 左转
- 桥接公式: `steering = 1500 - angular.z * gain` (符号取反)

## 手势系统

### 13. 手势检测码 — 需投票防抖
- OK(11), Palm(5), None(0)
- 单帧检测不可靠, 需连续 30 帧投票 (>0.5s) + 3s 冷却
- Palm 只在已锁定时才生效, 避免误解除

## 手势-人体匹配

### 14. 手势消息包含多种 ROI — 需过滤类型
- `/hobot_hand_gesture_detection` 的 targets 混合 body/head/face/hand 四种 ROI
- `_match_gesture_to_person` 必须只检查 `groi.type == 'hand'`，否则 body 中心点必然落在自身 rect 内导致误匹配

### 15. 投票循环不可遍历多 target
- 原代码处理第一个非零手势后立即 `return` 是故意的
- 移除 `return` 会导致后续 `gesture=0` 将刚积累的 votes[11] 清零，投票永远无法达到阈值 30

### 16. 无人检测需基于 body ROI 而非 person type
- `mono2d_body_detection` 消息中所有 target (body/head/face/hand) 的 type 均为 `'person'`
- `any(t.type == 'person')` 永远为 True，`_empty_since` 计时器永不启动
- 必须改为检查 `any(roi.type == 'body')` — `_has_body()`

### 17. 被锁者重识别 (Re-ID) — 空间匹配应对 ID 变化
- 检测器内部 tracker 在人短暂消失后会分配新 track_id
- perception_node (原 display_node) 的 HOLDING 保留了旧 ID 但人回来时拿到新 ID
- 解决: 记录锁定者最后已知 body 中心点, HOLDING 期间对新 body 做 150px 距离匹配, 匹配成功则更新 `_locked_id`

## 系统精简

### 18. websocket/nginx 冗余 — 用 HDMI 屏幕替代 Web 显示
- websocket 节点占 ~14% CPU + 69MB RAM, nginx 占 ~1MB
- 板端 HDMI 屏幕完全可以替代 Web 可视化
- 从 `person_follow.launch.py` 移除 websocket launch, EXPECTED_NODES 11→10

### 19. 桌面组件可安全清除
- xubuntu-desktop/gnome-shell/lightdm/xfce/snapd 等占 ~1.3GB 磁盘
- 清除后只保留 `xserver-xorg-core` + `xserver-xorg-video-fbdev` + `xinit` (OpenCV 显示必须)
- 关键: **必须在清除前 `apt-mark manual` 保护 xorg/xinit**, 否则 autoremove 会误删

### 20. 系统服务精简
- 安全禁用的 8 个服务: tftpd-hpa, accounts-daemon, lightdm, dnsmasq, auto_update_miniboot, hobot-suspend-button, rpcbind, udisks2
- 效果: 内存 used 531MB→250MB (idle), 服务 25→17 个

### 21. systemd 服务 KillMode 修复
- `Type=simple` + `ros2 launch` 子进程多, 默认 `KillMode=control-group` 导致 `systemctl restart` 永远等待子进程退出
- 修复: `/etc/systemd/system/tracked-vehicle-display.service` 添加 `KillMode=mixed` + `TimeoutStopSec=30`

### 22. udev symlink 在启动早期可能未就绪
- `/dev/stm32_board` symlink 由 udev 规则创建, 但服务启动可能早于 udev 触发
- `motor_bridge` (原 cmd_vel_bridge) 因无法打开串口而静默崩溃
- 临时方案: `ln -sf /dev/ttyUSB0 /dev/stm32_board` + 重启服务

## CI1302 语音模块

> 当前节点: `motion_arbiter.py` (原 `voice_bridge.py`, v0.7 重构)

### 23. CI1302 开机自动播报 "这是西瓜皮属于湿垃圾" — init 命令 ID 冲突

- **症状**: 每次 X5 重启, CI1302 语音模块自动播报 "这是西瓜皮，属于湿垃圾", 持续数月未能定位
- **根因**: `voice_bridge.__init__` 中 `self._write_cmd(0x67)` 向 CI1302 发送 `AA 55 FF 67 FB` 作为初始化命令。在出厂固件的协议表中, `0x67` 这个 CMD_ID 刚好被映射为垃圾分类演示播报（"这是西瓜皮，属于湿垃圾"）。每次 voice_bridge 启动 → 发送 init → 触发垃圾语音
- **排查难点**: 
  - 表象像"开机自动播报", 容易误判为 CI1302 固件的欢迎词设置问题
  - 曾花大量时间研究 CI1302 SDK、AI 平台、修改固件, 实际与固件无关
  - `0x67` 在 ROS1 参考代码中作为 init 命令使用, 但那是针对旧版固件; 新版固件的协议 ID 分配不同
- **修复**: 删除 `_write_cmd(0x67)` 调用。CI1302 上电后自动初始化, 不需要额外 init 命令
- **教训**:
  - 向外部设备发送命令前, 必须对照该设备的**实际协议表**确认命令 ID 的含义, 不能沿用旧代码的"惯例"
  - 排查问题时先确认是"主动触发"还是"被动自动"——一个简单的"停发命令"测试就能排除很多方向
  - 协议 ID 在不同固件版本间可能被重新分配, 升级/更换固件后必须重新核对
  - **后续**: 该问题最终通过升级到 V01843 固件 + DNN 分离唤醒词模型彻底解决 (见 #24, #25)

### 24. CI1302 固件升级 — 协议格式不兼容 (AA 55 → A5 FA)

- **症状**: 新固件刷入后, voice_bridge 完全不识别任何语音命令
- **根因**: V00681→V01843 固件使用完全不同的串口协议:
  - 旧: `AA 55 [STATUS] [CMD] FB` (5 bytes, 无校验)
  - 新: `A5 FA 00 [TYPE] [CMD] 00 [CKSUM] FB` (8 bytes, 累加和校验)
  - TYPE=0x81 (CI1302→Host), TYPE=0x82 (Host→CI1302)
  - CMD 映射重新编号: 停止 0x01→0x06, 前进 0x03→0x07, ...
- **修复**: 重写 voice_bridge.py 适配新协议 + 累加和校验 + 新 CMD 映射
- **教训**:
  - 升级固件后必须对照 SDK 源码中的协议表 (`send_data[]`/`recv_data[]`) 确认帧格式
  - 协议帧头、帧长、CMD 映射都可能变化, 不要假设向下兼容
  - voice_bridge 对无法识别的帧应输出 WARN 日志, 而不是静默丢弃

### 25. 唤醒词安全 — DNN 分离模型 vs 软件门控

- **需求**: 语音控制必须"先唤醒再命令", 不能直接喊指令就动
- **方案对比**:
  - 软件门控 (`get_wakeup_state()` 检查): UART 输出仅在 `sys_asr_result_hook` 绕过检查
  - **DNN 分离模型 (`USE_SEPARATE_WAKEUP_EN=1`)**: 唤醒前 ASR 只加载唤醒词模型, 命令词完全不被识别
- **最终方案**: V01843 固件启用 `USE_SEPARATE_WAKEUP_EN=1`, DNN 级安全隔离
- **教训**:
  - 软件门控不一定覆盖所有输出路径 (UART hook 是典型遗漏点)
  - DNN 级隔离是最强的安全保证 (模型根本不识别命令词)
  - 双层防御最可靠: 模型层 + 协议层各管一道门

### 26. CI1302 固件时钟源与波特率校准联动故障

- **症状**: 新固件刷入后, voice_bridge 正确发送欢迎语触发帧, CI1302 不播放音频, 串口返回数据始终乱码 (A5 FA 帧头正确但每字节有位错误)
- **排查过程**:
  - 怀疑 voice_bridge 协议帧格式错误 → 对照 readme.txt 确认帧 `A5 FA 00 82 02 00 23 FB` 完全正确
  - 尝试双协议兼容 (同时发 AA 55 + A5 FA) → 均不播放, 排除帧格式问题
  - 怀疑 PA / 扬声器硬件 → 旧固件曾成功播放音频, 排除
  - 扫描多个波特率 (9600-230400) → 全部无法获得有效帧
  - **关键突破**: 旧固件使用内部 RC + 波特率校准 → 通信正常; 新固件配置为外部晶振 + 波特率校准 → 通信乱码
- **根因**: 固件生成时两个配置项的组合错误:
  - `USE_EXTERNAL_CRYSTAL_OSC = 1`: CI1302 模块实际使用内部 RC 振荡器, 错误配置为外部晶振导致时钟基准偏离
  - `UART_BAUDRATE_CALIBRATE = 1`: 波特率校准在错误时钟基准上运行, 反而加大偏差
  - 两个错误叠加 → CI1302 实际波特率严重偏离 115200 → 收发双向乱码 → `recv_data[]` memcmp 匹配失败 → 欢迎语命令被丢弃
- **修复**: 重新生成固件: `USE_EXTERNAL_CRYSTAL_OSC = 0`, `UART_BAUDRATE_CALIBRATE = 0`
- **教训**:
  - 串口通信异常的**首要排查项**是波特率/时钟源配置, 不要直接怀疑协议帧格式
  - CI1302 模块 (ASR02M 等) 通常使用内部 RC 振荡器, `USE_EXTERNAL_CRYSTAL_OSC` 必须根据实际硬件设置, 不能照搬 SDK 默认值
  - `UART_BAUDRATE_CALIBRATE` 在时钟源正确时有助于补偿 RC 误差, 但在时钟源配置错误时反而加剧偏差
  - 排查时用原始串口收发测试 (python pyserial), 不要只依赖 ROS2 日志 (log_level=warn 会过滤 INFO 日志)

### 27. 欢迎语触发 — 事件驱动 vs 盲等定时器

> 当前节点: perception_node ↔ motion_arbiter (原 display_node / voice_bridge)

- **初始方案**: `welcome_delay_s` 参数控制 motion_arbiter 启动后延时 N 秒触发欢迎语
- **问题**: 欢迎语总是比屏幕 "ALL SYSTEMS GO" 早几秒或晚几秒:
  - perception_node 用硬编码 30s 判断系统就绪 (`_startup_done`)
  - motion_arbiter 用独立定时器, 两者无协调机制
  - 调整 `welcome_delay_s` 只能在特定启动速度下对齐, 冷启动/热启动时间不同时必然错位
- **正确方案**: perception_node 在 `_startup_done` 时 publish `/system_ready` (Bool), motion_arbiter 订阅该 topic, 收到后立即触发欢迎语
- **效果**: 欢迎语与 "ALL SYSTEMS GO" 精确同步 (<1s 偏差)
- **教训**:
  - 多个节点之间的时序协调应使用 **topic 事件驱动**, 不要各自维护独立定时器
  - 独立定时器在系统启动时间波动时必然错位, 事件驱动天然自适应
  - 发布-订阅模式是 ROS2 中最简单有效的跨节点同步机制

### 28. GS130W 双目深度 + 检测无法在 X5 上并发

- **需求**: 同时跑双目深度 (StereoNet) + 人体检测 (mono2d_body_det), 获取目标 3D 距离
- **结论**: **不可能**, 三重硬件限制:
  1. **mipi_cam 互斥**: 单通道 (960×544) vs 双通道 (640×352) 只能二选一, 无法共用一个摄像头实例
  2. **BPU 单核**: StereoNet 占 98-100% BPU, 检测模型无法获得推理时间
  3. **模型分辨率固定**: 检测模型 960×544, StereoNet 640×352, 无法统一输入尺寸
- **当前最优解**: 单目检测 (60FPS) + LiDAR 测距 (EKF 融合), 零 BPU 开销
- **硬件升级路径**: RGB-D 摄像头 (RealSense D435i / Orbbec Gemini) — 自带深度计算芯片, 不占 BPU
- **教训**: 做嵌入式视觉系统设计前, 必须先确认 BPU 核心数、模型分辨率要求、摄像头接口是否独占。事后发现限制比事前调研成本高得多
- **详见**: [stereo-depth-exploration.md](stereo-depth-exploration.md)

### 29. LiDAR 融合: 固定聚类阈值 → 自适应 + 躯干过滤

- **症状**: 墙壁/柱子被匹配为人体, 远距离人体因点稀疏而丢失聚类
- **根因**: 固定 0.20m 聚类阈值无法同时适应近距离 (点间距 ~1.5cm) 和远距离 (点间距 ~7cm+)
- **修复**:
  1. 自适应阈值: <1m:0.10m, <3m:0.20m, <6m:0.30m, >6m:0.40m (参考 Zhu Wang et al. 2021)
  2. 躯干几何过滤: 弧宽 15-70cm + 曲率 <0.97 排除墙壁/柱子
  3. 贪心匹配 → 匈牙利全局匹配 (scipy linear_sum_assignment)
- **验证**: 13 个聚类 → 10 躯干 + 3 墙壁, 匹配池不再被墙壁污染
- **教训**: 2D LiDAR 在胸高度 (~150cm) 扫描人体是连续弧面, 与腿高度 (两根分离腿柱) 完全不同。Leg Detection 方案 (mowito/ros2_leg_detector) 不适用于胸高度安装

### 30. LiDAR EKF: Q 矩阵必须按 dt 缩放

- **症状**: 60Hz predict 每次累加恒定 Q, 导致 10Hz LiDAR 帧间注入 ~6x 过多过程噪声
- **修复**: `self.P = F @ self.P @ F.T + self.Q * dt`, Q 基准值上调 10x 补偿
- **教训**: 离散时间 EKF 中, 过程噪声协方差必须缩放至预测步长。渲染帧 (60Hz) 比 LiDAR 帧 (10Hz) 高 6 倍, Q 累加 6 次导致协方差膨胀和滤波器过度信任新测量

### 31. Camera FOV 校准: 广角镜头标定文件正确性验证

- **症状**: 感觉摄像头 FOV ">120°", 担心 72° 硬编码不准确
- **验证**: 从 GS130W 规格 (DFOV=157.2°, f=1.75mm) 和 SC132GS 校准文件 (fx=656.76 -> f=1.76mm) 确认匹配
- **结论**: 72° 是 GDC 去畸变+裁剪后的有效总 HFOV (±36°), 硬编码正确。感知到的 ">120°" 是对角线 FOV (157.2°)
- **教训**: `camera_name: narrow_stereo` 命名误导 — 校准数据的实际焦距与 GS130W 广角规格吻合, 不能仅凭文件名判断

### 32. motion_arbiter 速度映射: 离散区间 → 连续曲线

- **症状**: 0.7m 边界处速度从 -0.2 跳变到 0 m/s, 造成边界震颤
- **修复**: 0.7-0.85m 增加 15cm 线性过渡区, 1.2-3.0m 用二次曲线加速 (0 -> 0.8 m/s)
- **附加**: vel_fast 0.3→0.8 m/s (跟上步行), dist_far 2.5→3.0m
- **教训**: 履带车辆的运动控制必须避免离散区间跳变, 惯性+延迟使边界震荡不可避免

### 33. 语音指令后 X5 整机硬重启 — 电机浪涌欠压 (brownout)

- **症状** (v0.8.0 现场): 给出语音运动指令后系统崩溃重启 — HDMI 黑屏, 整机重新走启动流程
- **诊断过程**:
  1. `uptime` 仅 1 分钟 → 确实发生了整机重启 (非单个节点崩溃)
  2. 上一 boot 的 journal 在 body_tracking 60Hz 日志流中**戛然而止** — 无任何 shutdown/SIGTERM/"process has died" 记录 → **硬复位 (掉电), 非软件崩溃**
  3. systemd `Restart=on-failure` 无触发记录, dmesg 无 thermal/watchdog 痕迹, 无 Python traceback
  4. 语音指令 = 唯一同时触发 "CI1302 播报 + 双 BLDC 从静止启动" 的场景 — 两者瞬时电流叠加拉低母线
- **根因 (高置信度)**: 重型履带车电机启动浪涌 + 语音播报电流 → 母线电压跌落 → X5 欠压硬复位. 与软件改动无直接因果 (同期改动仅渲染降频)
- **修复**: STM32 PWM 斜率软启动 (加速限速 1200μs/s, 减速/急停不限) + 排查供电链 (电池电量/线径/接头压降)
- **教训**:
  - 整机"重启"先区分软件崩溃 vs 硬复位: journal 尾部戛然而止 = 掉电; 有 died/traceback = 软件
  - body_tracking 的 60Hz WARN 日志会刷爆 journal 淹没现场证据 — launch 中已固定为 error 级
  - 语音运动指令只发单帧 → STM32 2s 超时即停车 (动作时长不足); motion_arbiter 已改为 3s 窗口内 5Hz 重发
  - STM32 的调试打印此前无人读取, 现场完全盲区; motor_bridge 现已转发关键事件到 ROS 日志

### 34. SBUS CH6 模式切换通道抖动 — 四层故障链与纵深防御

- **症状** (v0.8.1 现场): CH6 未拨动时模式在手控/X5 间抖动, 蜂鸣器反复提示切换
- **根因 (按贡献排序)**:
  1. **帧同步只靠狩猎 0x0F, 且无帧尾校验** (WFLY byte24 非标被移除) — 通道数据字节中
     的 0x0F 会触发假帧; 一旦丢字节错位, 稳态杆位下错位**自持续**, 每帧解出一致的错值.
     中值滤波对"一致错值"完全无效, 必须在帧同步层解决
  2. **5Hz 状态打印阻塞 ~7ms** (80B @115200), 期间 USART2 仅 1 字节 HW 缓冲 → ORE 丢字节
     → 错位 → 突发垃圾帧. 每 200ms 制造一次丢帧窗口
  3. ESC PWM 开关噪声耦合进 SBUS 走线 → USART2 FE/NE/PE 错误字节
  4. CH6 三档开关机械切换经过中位 (~992), 恰好跨裸阈值 1024
- **修复 (纵深防御, 由低到高)**:
  - 帧同步: 0x0F 前需 ≥1ms 空闲间隔 (帧内字节 ~110μs 连续, 帧间 ~11ms) — 数据字节
    0x0F 不再触发假帧, 从根上消除错位锁定
  - 错误处理: ORE 丢字节 → 整帧丢弃并计数; FE/NE/PE 仅清标志 (数据仍可用)
  - 通道滤波: 5 帧 (~70ms) 中值滤波, 按帧同步采样而非 loop 迭代
  - 滞回: Schmitt 死区 >1500 自动 / <600 手控, 覆盖三档开关中位
  - 时间确认: 目标态需稳定 3s 才切换; 仲裁 gate 在 sbusActive 上 (失控冻结, 不静默翻回手控)
  - 诊断: 状态行输出 ore=/hdr= 计数器 + [DBG] CH6 跳变打印, 现场可定位噪声来源
- **同类 bug**: CH5 "3帧防抖" 实际按 loop 迭代计数 (~μs 级), 3 次迭代 <1ms 即饱和,
  等于无防抖 — 一段 14ms 毛刺窗口即可触发假 ARM/DISARM. 已统一改为帧同步采样
- **教训**:
  - 无 CRC/帧尾的协议 (SBUS), 帧同步必须依赖物理层特征 (空闲间隔), 不能靠内容匹配
  - "N 帧防抖"若按循环迭代计数, 在快速循环里等于零防抖 — 采样必须同步到数据到达事件
  - 中值滤波只隔离随机尖峰, 对系统性错位 (一致错值) 无效 — 滤波不能替代帧完整性

### 35. X5 模式原地剧烈抖动 <5cm — uint32 时间戳下溢 (潜伏自 v0.8.1)

- **症状** (2026-07-21 现场): X5 模式给前进指令, 车原地剧烈抖动, 位移 <5cm; 手控正常
- **取证 (bypass 隔离测试, 推荐复用)**: 停 ROS 服务, X5 命令行直接写串口发 20Hz
  thr=1700 帧 3 秒 → 60 帧 100% 解析 (排除 ROS/CH340/线束), 但 [SAFE]超时/[X5]恢复
  仍交替刷屏 → 锁定固件逻辑层
- **根因**: loop() 顶部 `now=millis()` → sbusPoll() 阻塞 ~2.4ms → x5ParseMotorCmd()
  用阻塞后的 `millis()` 打戳 `g_lastX5Ms` → 同轮 `now - g_lastX5Ms` 为负 →
  uint32 下溢 ~42 亿 > 2000ms → 恒判 X5 超时 (本轮输出中位), 下轮恢复 →
  每帧一次 超时/恢复 循环, PWM 以 ~5Hz 振荡. 定量吻合: SBUS 阻塞概率 ×
  MotorCmd 帧率 ≈ 4-5 对/秒 = 日志观测频率
- **修复 (一行)**: 控制仲裁前 `now = millis()` 刷新. 打戳与比较必须同一时刻源
- **为何潜伏至今**: 手控分支不用该时间戳; v0.8.1 测试被 brownout 打断;
  motor_bridge 加 20Hz keepalive 连续帧流后才显性化
- **教训**:
  - uint32 时间戳比较, 打戳与读取必须同一时刻源; 跨阻塞调用混用 = 下溢
  - 多层系统丢帧排查: bypass 隔离测试一次定位到单层, 远胜逐层猜
  - "日志里链路不稳定" ≠ 链路丢帧 — 解析计数器 (ore=/hdr=) 才能区分

### 36. WFLY 遥控器 SBUS 校准 — raw 中位 1024 ≠ FrSky 992

- **症状**: 手控满量程改造后, 不打杆车缓慢右转 (回中输出固定 +32μs)
- **根因**: WFLY raw 中位 = 1024 (遥控器 µs 显示 1500 = raw 1024),
  CH6 三档实测 352/1024/1695; 代码按 FrSky 标准 992 校准
- **修复**: SBUS_CENTER=1024, SBUS_RAW_HALF_SPAN=672, 满杆精确 ±500μs
- **教训**: 换遥控器/接收机必须实测 raw 值校准; 状态行加 c1=/c2= 原始值
  输出可现场确认, 不要假设"标准"中位

---

## v0.9.0 经验

### 37. 锁漂移根因 — RE-ID 在第一帧触发, HOLDING 状态形同虚设

- **症状**: 多人画面中锁定 A 人, B 人路过遮挡 A 不到半秒, 锁定自动切到 B
- **根因**: `_target_visible()` 返回 false → `_lost_since` 刚设为 now → **同一帧内立即调用 `_find_nearest_body()`** → 150px 内找到 B → `_locked_id` 切到 B, `_lost_since` 重置。5 秒 HOLDING 超时逻辑在代码后面, 但永远到不了 — RE-ID 在第一步就成功并重置了计时器
- **修复**: 加 `_lost_reid_min_s=1.0s` 保持窗口 — 丢失 1s 后**才**尝试 RE-ID, 缩小搜索半径到 80px
- **教训**: 状态机审查必须逐分支走查, 看似完整的 IDLE→LOCKED→HOLDING 三段式, 实际 HOLDING 只在 RE-ID 失败后才触发; RE-ID 成功直接绕过了整个保护层

### 38. 后退犹豫 — 15cm 过渡区 + 无迟滞 + 静摩擦地板

- **症状**: 人走向车辆时后退犹豫/不动, 远不如前进丝滑
- **根因 (三层)**:
  1. 后退过渡区仅 0.70-0.85m (15cm), LiDAR σ≈3cm → 距离在边界跳动 → 速度在-0.3↔0 反复切换
  2. 无迟滞: 进出后退区使用同一阈值, 产生 chatter
  3. 履带车静摩擦阈值约 -0.15 m/s: 过渡区上段 (0.78-0.85m) 计算的 -0.08~-0.14 m/s 不足以克服静摩擦
- **修复**: Schmitt 迟滞 (进<0.85m/出>1.0m) + 速度地板 -0.15 + 0.5m 分段渐变
- **教训**: 对称的进出阈值对噪声敏感的传感器是致命的; 任何边界检测都应使用迟滞

### 39. 纯 P 控制对重载履带车必然震荡

- **症状**: 前进跟随时左右来回修正不收敛, 12s 周期持续震荡
- **根因**: `angular.z = -0.5 * y` — 纯比例控制, 阻尼比 ζ=0, 物理意义上是无阻尼振荡器。对重载履带车 (15kg, 大惯性, skid-steer 非线性摩擦), P 控制的相位滞后更严重
- **修复**: PD 控制 (k_p=0.4, k_d=1.2, ζ≈0.65) + ±5cm 死区 + 低通滤波 α=0.25
- **教训**: 嵌入式机器人控制中, 纯 P 控制几乎永远不够 — 系统惯性和传感器延迟 (>100ms LiDAR) 叠加后相位裕度极差

### 40. 手势模型不输出 confidence — attr.score 字段不存在

- **症状**: 加了置信度门控 `attr.score >= 0.5` 后 perception_node 启动即 crash
- **根因**: Horizon `PerceptionTargets.Attribute` 消息的字段是 `confidence` 不是 `score`, 且 gestureDet_8x21 模型输出 confidence=0.0 (不填充)
- **修复**: 改用 `attr.confidence`, 默认阈值 0.0 (相当于禁用)
- **教训**: 跨厂商 SDK 的 ROS 消息字段, 不要凭直觉假设命名; 必须先读 `.msg` 定义文件

### 41. 日志炸弹 — body_tracking @ WARN 可刷出 1.9GB

- **症状**: /root/.ros/log/ 持续增长, syslog 631MB, journal 103MB
- **根因**: body_tracking 节点内部 rotate/cancel 状态切换 @ 60Hz WARN 级别; motor_bridge STM32-PWM 周期日志 @ 2Hz WARN
- **修复**: motor_bridge 周期状态改 DEBUG, 清理历史日志 2.5GB, journal vacuum 50MB
- **教训**: 第三方节点的日志级别需从第一天就锁定; ROS 日志文件 (`/root/.ros/log/`) 无自动 rotation, 比 journal 更危险

### 42. 系统启动 62s — apt-show-versions 独占 45s

- **症状**: 机器人上电到服务启动需 1 分钟以上
- **根因**: `apt-show-versions.service` (45s) + `hobot-switch-aptsource.service` (23s) 在嵌入式设备上完全无意义
- **修复**: disable + mask 8 个无用服务, 启动降到 20s
- **教训**: 嵌入式 Linux 出厂镜像带了大量桌面/server 级 systemd 服务, 首次部署应做 `systemd-analyze blame` 审计
