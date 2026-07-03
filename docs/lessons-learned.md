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
- display_node 的 HOLDING 保留了旧 ID 但人回来时拿到新 ID
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
- `cmd_vel_bridge` 因无法打开串口而静默崩溃
- 临时方案: `ln -sf /dev/ttyUSB0 /dev/stm32_board` + 重启服务

## CI1302 语音模块

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

- **初始方案**: `welcome_delay_s` 参数控制 voice_bridge 启动后延时 N 秒触发欢迎语
- **问题**: 欢迎语总是比屏幕 "ALL SYSTEMS GO" 早几秒或晚几秒:
  - display_node 用硬编码 30s 判断系统就绪 (`_startup_done`)
  - voice_bridge 用独立定时器, 两者无协调机制
  - 调整 `welcome_delay_s` 只能在特定启动速度下对齐, 冷启动/热启动时间不同时必然错位
- **正确方案**: display_node 在 `_startup_done` 时 publish `/system_ready` (Bool), voice_bridge 订阅该 topic, 收到后立即触发欢迎语
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
