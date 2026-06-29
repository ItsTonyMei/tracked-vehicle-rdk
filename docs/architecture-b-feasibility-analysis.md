# Architecture B 可行性分析：RDK X5 + GS130W 双目深度 + 人体检测融合

> 分析日期: 2026-06-29
> 项目: tracked-vehicle-rdk (v0.5.1)
> 作者: Hermes Agent (基于项目代码审查 + 官方文档 + 论坛数据)

---

## 1. 背景

6WD 履带跟随车项目已从 ESP32+OpenMV 迁移至 RDK X5 + STM32 架构，当前达成：

| 子系统 | 状态 | 说明 |
|--------|------|------|
| STM32 固件 | ✅ | SBUS + MotorCmd + 坦克混控 + IWDG + 7层安全 |
| 双目深度验证 | ✅ | GS130W(SC132GS) + StereoNet V2.4_int8 @ 21.3fps |
| 人体跟随管线 | ✅ | TROS 10节点: mono2d_body_det → hand_lmk → gesture → body_tracking → bridge |
| HDMI 屏显 | ✅ | 1024×600 + 手势锁定状态机 + 系统监控 |
| MotorCmd 桥接 | ✅ | /cmd_vel → CRC-8 6字节帧 → STM32 |

**缺失**: 当前跟随系统使用**单目距离估计**（bbox宽度反比），未接入双目深度数据。
架构 B（左目检测 + 双目测距同时运行）是项目的目标架构，尚未实现。

---

## 2. 关键发现：BPU 瓶颈

### 2.1 StereoNet 的 BPU 占用

来源: `docs/stereo-vision-verification.md`（实测数据）

```
模型:     DStereoV2.4_int8
输入:     640×352×3×2 (双目)
帧率:     21.3 fps
延迟:     144-178ms
BPU 占用: 98-100%    ← 致命
CPU 占用: 113-126%
```

### 2.2 人体检测的 BPU 需求

```
模型:     mono2d_body_detection (multitask_body_head_face_hand_kps)
输入:     960×544 (单目左通道)
帧率:     60 fps (官方标称)
推理:     BPU 加速
```

### 2.3 冲突

RDK X5 的 BPU (Bernoulli 架构, 10 TOPS) 是**单推理引擎**——不支持两个模型同时加载执行。StereoNet 单独已占满 BPU，无余量运行检测模型。

这意味着架构 B 的理想形态（两个 BPU 模型并行推理）在当前硬件上**不可行**。

---

## 3. 三条可行路径

### 路径 1: CPU 检测 + BPU 专跑深度（推荐）

```
┌─────────────────────────────────────────────────────────┐
│  GS130W MIPI 双路采集 (VSE 硬矫正, 不占 BPU)              │
│                                                         │
│  ┌──────────────────┐    ┌─────────────────────────┐    │
│  │ 左目 640×352     │    │ 左+右 640×352×2          │    │
│  │       │          │    │       │                  │    │
│  │       ▼          │    │       ▼                  │    │
│  │  CPU 检测        │    │  BPU StereoNet          │    │
│  │  NanoDet-Plus    │    │  V2.4_int8 @ 21fps      │    │
│  │  A55 × 2-3核     │    │  BPU 98%               │    │
│  │  预计 12-18fps   │    │                         │    │
│  │       │          │    │       │                  │    │
│  │       ▼          │    │       ▼                  │    │
│  │  detection_boxes │    │  depth_map              │    │
│  │  (track_id, x,y, │    │  (mm, per-pixel)        │    │
│  │   w,h, class)    │    │                         │    │
│  └────────┬─────────┘    └───────────┬─────────────┘    │
│           │                          │                  │
│           └──────────┬───────────────┘                  │
│                      ▼                                  │
│              融合节点 (stereo_fusion_node)                │
│              ROI 深度采样 → Person3D                     │
│                      │                                  │
│                      ▼                                  │
│              follow_logic (distScore)                   │
│                      │                                  │
│                      ▼                                  │
│              /cmd_vel → MotorCmd → STM32                │
└─────────────────────────────────────────────────────────┘
```

**优点**:
- 零 BPU 调度冲突，StereoNet 不受影响
- RDK X5 8核 A55 当前只用 ~1.3核（126% CPU），余量充足
- 开发周期短（3-5天），风险极低
- 检测和深度完全解耦，各自独立调试

**缺点**:
- CPU 检测帧率可能低于 BPU 检测（12-18fps vs 60fps）
- 对履带车跟随（<2m/s）12fps 仍然够用，延迟 <100ms 对应 <0.2m 位置误差

**工作量估算**:

| 任务 | 工时 | 产出 |
|------|------|------|
| NanoDet-Plus 模型准备（ONNX 转换 + A55 优化） | 0.5天 | `.onnx` 模型文件 |
| CPU 检测 ROS2 节点 (`cpu_detector.py`) | 1天 | 发布 `/cpu_detections` topic |
| 融合节点 (`stereo_fusion_node.py`) | 1天 | 订阅 depth + detections → Person3D |
| 集成 launch + 屏显适配 | 0.5天 | 修改 person_follow.launch.py |
| 实测调优 | 1天 | 帧率、延迟、精度验证 |
| **合计** | **4天** | |

---

### 路径 2: BPU 时间分片（需要地平线 FAE 支持）

```
BPU 时间轴 (以 8 帧为一个周期, ~350ms):
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ S    │ S    │ S    │ S    │ S    │ S    │ D    │ S    │
│Stereo│Stereo│Stereo│Stereo│Stereo│Stereo│Detect│Stereo│...
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 21fps  21fps  21fps  21fps  21fps  21fps  YOLO   21fps

有效深度帧率: ~24fps (7/8帧跑深度)
有效检测帧率: ~3fps  (1/8帧跑检测)
```

**关键前提（必须地平线确认）**:
1. BPU 运行时能否通过 API 卸载当前模型并加载另一个 `.hbm`？
2. 模型切换延迟是多少？（可能 50-200ms）
3. 频繁切换是否影响 BPU 稳定性/寿命？

**优点**:
- 深度帧率几乎无损（24fps vs 21fps）
- 检测也跑在 BPU 上（虽然帧率低）

**缺点**:
- 检测仅 3fps——快速横移的目标可能漏检
- 需要地平线官方确认 API 可行性，可能发现不可行
- 切换开销可能吃掉收益
- 开发周期长，调试困难

**工作量估算**: 1-2周（包括 FAE 沟通 + 大量测试），且可能因 API 不支持而走不通。

---

### 路径 3: 单模型联合推理（研究性质，不推荐当前阶段）

通过地平线工具链 (hb_mapper) 将检测头和视差头合并到一个 HBM 模型：
- 共享 backbone → 双头 (detection head + disparity head)
- 需要联合训练或分阶段训练

**障碍**:
- 地平线工具链对自定义多任务模型的支持程度未知
- 10 TOPS BPU 是否够同时出检测+视差
- 模型精度需要大规模验证
- 工作量: >1个月，且可能最终不可行

**结论**: 当前阶段不考虑。

---

## 4. 推荐方案: 路径 1 详细设计

### 4.1 新增文件

```
src/tracked_vehicle/tracked_vehicle/
├── cpu_detector.py          # CPU 人体检测 ROS2 节点
├── stereo_fusion_node.py    # 深度融合节点 (detection + depth → Person3D)
└── follow_logic_node.py     # (可选) distScore 跟随算法节点

launch/
└── person_follow_stereo.launch.py  # 新 launch: 双目跟随全管线
```

### 4.2 cpu_detector.py 设计

```python
# 输入: /image_combine_raw/left (或复用现有 /image topic)
# 输出: /cpu_detections (自定义 Detection3D msg 或复用 ai_msgs/PerceptionTargets)
# 模型: NanoDet-Plus-m (ONNX, onnxruntime, ~1.8M 参数)
# 帧率目标: 12-18fps @ 640×352
# CPU 核: 2-3 核 (通过 thread affinity 或 taskset 固定)

class CpuDetector(Node):
    def __init__(self):
        # 加载 ONNX 模型
        self.session = ort.InferenceSession('nanodet_plus_m_320.onnx', 
                                             providers=['CPUExecutionProvider'])
        # 订阅左目图像
        self.sub = self.create_subscription(Image, '/left_image', ...)
        # 发布检测结果
        self.pub = self.create_publisher(DetectionArray, '/cpu_detections', 10)
```

### 4.3 stereo_fusion_node.py 设计

```python
# 输入:
#   /cpu_detections (DetectionArray) — 人体检测框
#   /stereonet_depth (Image, 16UC1)  — 深度图 (单位 mm)
# 输出:
#   /person_3d (Person3DArray) — 人体 3D 位置
#   /cmd_vel (Twist)          — (可选, 直接在此节点做跟随决策)

class StereoFusionNode(Node):
    def __init__(self):
        # 订阅深度图
        self.sub_depth = self.create_subscription(
            Image, '/stereonet_depth', self.depth_cb, BEST_EFFORT(1))
        # 订阅检测框
        self.sub_det = self.create_subscription(
            DetectionArray, '/cpu_detections', self.det_cb, 10)
        
        # 相机内参 (来自 stereo_calib.yaml)
        self.fx = 491.73
        self.fy = 491.93
        self.cx = 406.33
        self.cy = 482.29
    
    def fuse(self, boxes, depth_map):
        """对每个检测框在深度图 ROI 内采样, 输出 3D 位置"""
        results = []
        for box in boxes:
            x1, y1, x2, y2 = box
            roi = depth_map[y1:y2, x1:x2]
            valid = roi[roi > 0]
            if len(valid) < 10:
                continue  # 该框内有效深度点太少, 跳过
            z = np.median(valid) / 1000.0  # mm → m
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            x = (cx - self.cx) * z / self.fx
            y = (cy - self.cy) * z / self.fy
            results.append(Person3D(track_id=box.track_id, x=x, y=y, z=z))
        return results
```

### 4.4 距离精度预期

```
StereoNet + GS130W @ 基线=60mm, 焦距≈492px:

距离(m)   视差(px)   1px误差→距离误差   测量精度
  1         29           ±0.03m          ±3cm
  3         10           ±0.3m           ±30cm
  5          6           ±0.8m           ±80cm
 10          3           ±3.3m           ±3.3m

有效范围: <5m (跟随场景主工作区)
>5m 时退化到单目作为 fallback
```

### 4.5 集成到现有 launch

`person_follow_stereo.launch.py` 合并两个管线:

```python
# ── 双目深度管线 (复用 stereo_vision.launch.py) ──
# mipi_cam_dual (640×352) + StereoNet V2.4_int8

# ── CPU 检测管线 (替代 mono2d_body_detection) ──
# cpu_detector (640×352) → 发布 /cpu_detections

# ── 融合 + 跟随 ──
# stereo_fusion_node → /person_3d → follow_logic → /cmd_vel

# ── 屏显 + 桥接 (不变) ──
# display_node + cmd_vel_bridge
```

---

## 5. 风险与缓解

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|----------|
| CPU 检测 <10fps | 中 | 跟随延迟 >100ms | 换更轻量模型 (NanoDet-Plus-m → NanoDet-Plus-s); 降分辨率到 320×176 |
| 深度图 ROI 有效点不足 | 高 | 输出距离不稳定 | 扩大采样区域(框外扩20%); 多帧时域平滑; 单目 fallback |
| 标定不精确导致 3D 位置偏移 | 高 | 跟随点偏移, 追偏 | 重新做精密双目标定 (棋盘格, >30对图像) |
| 光照变化导致深度失效 | 中 | 室外阴影区深度为0 | 深度置信度阈值过滤; 单目 fallback |
| CPU 发热 → 降频 | 低 | 帧率下降 | 固定 CPU 频率 (ondemand → performance governor) |

---

## 6. 与其他平台对比

| 平台 | 检测+深度方案 | 优点 | 缺点 |
|------|-------------|------|------|
| RDK X5 (路径1) | CPU NanoDet + BPU StereoNet | 低功耗(15W), 已验证硬件, 现有代码基础 | 检测帧率受限 |
| Jetson Orin Nano | GPU TensorRT YOLO + GPU Stereo | 高帧率(30fps+), 真并行 | 功耗高(15-30W), 价格2-3倍, 需重新适配 |
| ESP32+OpenMV (原方案) | 交替执行 | 超低功耗(<5W) | 帧率极低(2-5fps), 精度差 |
| RDK Ultra | BPU 96 TOPS 单模型联合 | 真并行, 高精度 | 价格4-5倍, 过度配置 |

**结论**: RDK X5 路径 1 是当前阶段最优解——利用已有硬件和代码，以最小改动实现双目测距跟随。

---

## 7. 下一步行动

### 立即执行 (本周)
1. [ ] 准备 NanoDet-Plus ONNX 模型 (选 NanoDet-Plus-m-320, COCO pretrained, 只取 person 类)
2. [ ] 编写 `cpu_detector.py` — 独立测试帧率
3. [ ] 确认 `/stereonet_depth` topic 名称和消息格式（当前 hobot_stereonet 是否发布深度图 topic）

### 短期 (下周)
4. [ ] 编写 `stereo_fusion_node.py` — 离线测试融合逻辑
5. [ ] 集成 launch + 屏显适配
6. [ ] 室内静态测距验证（1m/2m/3m/5m 定点测试）

### 中期 (2周内)
7. [ ] 室外实车测试
8. [ ] 距离精度对比 (立体测距 vs 单目估计 vs 激光雷达 ground truth)
9. [ ] 调优: 滤波、ROI 策略、fallback 逻辑

---

## 附录 A: 关键数据来源

| 数据 | 来源 |
|------|------|
| StereoNet BPU 占用 98-100% | `docs/stereo-vision-verification.md` 实测 |
| GS130W 基线 60mm | `config/stereo_calib.yaml` |
| 相机内参 fx=491.73 | `config/stereo_calib.yaml` |
| 当前单目距离公式 | `display_node.py` L414: `dist = bbox_ref * bbox_ref_dist / width` |
| body_tracking FPS 60 | `CHANGELOG.md` v0.3.0 |
| CPU 占用 113-126% | `docs/stereo-vision-verification.md` (stereo单独运行时) |
| 论坛案例: 标定不精确 → 深度差 | 帖子 #266442473063437798 (2025-03) |

## 附录 B: 官方资源索引

- 双目算法文档: https://developer.d-robotics.cc/rdk_doc/Robot_development/boxs/spatial/hobot_stereonet
- StereoNet GitHub: https://github.com/D-Robotics/hobot_stereonet
- Magicbox 双目深度: https://developer.d-robotics.cc/magicbox_doc/algorithm-development/stereo-depth
- B站教学视频: https://www.bilibili.com/video/BV1KdEjzREMz
- 论坛: https://developer.d-robotics.cc/forum
- 模型版本说明: V2.4_int8(23fps), V2.4_int16(15fps), V2.5_int16_96(18fps)
