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
