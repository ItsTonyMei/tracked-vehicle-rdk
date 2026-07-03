# 双目深度方案技术探索

> 2026-07-03 — 探索在 RDK X5 上同时运行双目深度 + 人体检测的可行性

## 背景

项目当前使用 GS130W 双目摄像头，但仅启用**单目模式**（左眼 960×544 @ 60FPS）跑人体检测。距离估算依赖 bbox 宽度公式 `dist = (bbox_ref × bbox_ref_dist) / bbox_width` + LiDAR 融合。

希望在不牺牲检测帧率的前提下，利用双目摄像头获取精确的 3D 深度数据。

## 板端环境

| 组件 | 版本 |
|------|------|
| RDK X5 OS | Ubuntu 22.04 (Jammy) |
| mipi_cam | tros-humble-mipi-cam 2.5.2 |
| hobot_stereonet | tros-humble-hobot-stereonet 2.5.5 |
| mono2d_body_detection | tros-humble-mono2d-body-detection |
| StereoNet 模型 | dstereo_s100_320_640_352_v2.4.hbm (640×352) |
| 人体检测模型 | multitask_body_head_face_hand_kps_960x544.hbm (960×544) |

## 核心限制

### 1. mipi_cam 互斥

mipi_cam 只能启动一个实例。当前 person_follow 管线使用**单通道模式** (960×544)，双目管线需要**双通道模式** (640×352×2)。两者 mipi_cam 配置不同，无法在同一个 launch 中共存。

### 2. BPU 单核争抢

RDK X5 的 BPU 3.0 只有一个物理核心：

| 模型 | FPS | BPU 占用 |
|------|-----|----------|
| StereoNet V2.4_int8 (双目深度) | ~21 | **98-100%** |
| Body Det (人体检测) | ~60 | ~40-50% |
| 同时运行 | 各 ~10-12 | 100% (争抢) |

StereoNet 吃满 BPU 后，检测模型几乎无法得到推理时间。

### 3. 模型分辨率不兼容

- 人体检测模型锁定 960×544 输入
- StereoNet 模型锁定 640×352（或 640×532）输入
- 共用一个 mipi_cam 时无法同时满足两个分辨率

## 已尝试方案

### stereo_depth_fusion 节点（已创建但未启用）

在 commit `78d395e` 中实现了 [stereo_depth_fusion.py](../src/tracked_vehicle/tracked_vehicle/stereo_depth_fusion.py)：

- 订阅 `/StereoNetNode/stereonet_depth`（16-bit 深度图, mm）
- 订阅 `/hobot_mono2d_body_detection`（人体检测框）
- bbox 中心区域深度采样 → 中值滤波 → 3D 距离
- 发布 `/person_distance`（track_id + x/y/z）

但受限于上述 mipi_cam 和 BPU 限制，该节点无法与 person_follow 管线**同时**运行。仅可作为分时使用的深度校准工具。

## 可行替代方案

### 方案 A：当前方案（已生效，推荐）

```
单目检测 (60FPS) + LiDAR 测距
```

- GS130W 单通道 960×544 → 人体检测 60FPS
- T-mini Plus 激光雷达 → [lidar_fusion.py](../src/tracked_vehicle/tracked_vehicle/lidar_fusion.py) EKF 融合
- 零 BPU 开销，零 mipi_cam 冲突
- 距离精度依赖 LiDAR（12m 范围, 毫米级）

### 方案 B：外接 RGB-D 摄像头（推荐升级路径）

| 型号 | 深度原理 | BPU 占用 | 估计成本 |
|------|----------|----------|----------|
| Intel RealSense D435i | 自带 ASIC 红外结构光 | **0%** | ~2500 元 |
| Orbbec Gemini 2 | 自带芯片 | **0%** | ~2000 元 |

RGB-D 摄像头在**内部芯片**完成深度计算，直接输出对齐的 RGB 图 + 深度图。X5 的 BPU 只需跑人体检测（60FPS 不变），同时从 ROS topic 获取现成深度数据。

集成方式：
- 深度图通过 ROS2 topic 发布（如 `/camera/depth/image_rect_raw`）
- `stereo_depth_fusion.py` 节点稍作适配即可消费
- 不需要 mipi_cam（通过 USB 3.0 连接）
- 不需要 StereoNet（不占 BPU）

## 结论

**GS130W 双目摄像头在 RDK X5 上无法同时实现全帧率深度 + 检测**，根本原因是 BPU 单核 + 模型分辨率不兼容 + mipi_cam 互斥。

当前 **GS130W 单目 + LiDAR** 方案已满足履带跟随车的距离测量需求。如需视觉级精确深度，推荐外接 RealSense D435i 等 RGB-D 摄像头，零 BPU 开销。
