# 双目视觉验证文档 — GS130W + StereoNet V2.4_int8

> 状态: ✅ 已验证通过  
> 日期: 2026-06-19  
> 设备: RDK X5 (RDK OS 3.5.0)  
> 相机: GS130W (SC132GS) MIPI 双目

---

## 验证目标

在 RDK X5 上验证 GS130W 双目摄像头 + StereoNet 深度估计的完整 pipeline：
- 双目图像采集 (mipi_cam)
- 立体匹配深度推理 (StereoNet V2.4_int8, BPU 加速)
- Web 实时可视化

---

## 最终工作参数

### mipi_cam

| 参数 | 值 | 说明 |
|------|-----|------|
| `mipi_image_width` | 640 | 匹配 stereonet 输入尺寸 |
| `mipi_image_height` | 352 | 匹配 stereonet 输入尺寸 |
| `mipi_image_framerate` | 30.0 | 相机采集帧率 |
| `mipi_out_format` | nv12 | BPU 原生格式 |
| `mipi_gdc_enable` | True | 几何畸变校正 |
| `mipi_rotation` | 90.0 | **必须**，SC132GS 竖屏 sensor |
| `mipi_cal_rotation` | 90.0 | 标定旋转 |
| `mipi_channel` | 0 | 左目 (从 2 改为 0，修正左右顺序) |
| `mipi_channel2` | 2 | 右目 |
| `mipi_lpwm_enable` | True | 低功耗模式 |
| `mipi_frame_ts_type` | realtime | 实时时间戳 |

### stereonet

| 参数 | 值 | 说明 |
|------|-----|------|
| 模型 | DStereoV2.4_int8 | 23fps 上限，int8 量化 |
| `camera_fx` | 491.73 | 缩放到 640×352 |
| `camera_fy` | 491.93 | |
| `camera_cx` | 406.33 | |
| `camera_cy` | 482.29 | |
| `baseline` | 0.06 | GS130W 典型基线 (米) |
| `render_type` | distance | 深度图渲染 |
| `render_max_disp` | 80 | 最大视差 |
| `render_z_range` | 3.0 | 深度范围 (米) |
| `infer_thread_num` | 2 | BPU 推理线程 |

---

## 性能指标

| 指标 | V2.5_int16 (旧) | V2.4_int8 (当前) |
|------|-----------------|-------------------|
| FPS | 15.7 | **21.3** |
| 延迟 | ~200ms | **144-178ms** |
| BPU 占用 | 91% | 98-100% |
| CPU 占用 | 80% | 113-126% |

---

## 踩坑记录

### 1. 画面 90° 翻转
- **症状**: Web 页面画面旋转 90°
- **根因**: SC132GS sensor 原生竖屏输出 (1088×1280)，mipi_cam 默认 rotation=0
- **修复**: 添加 `mipi_rotation:=90.0 mipi_cal_rotation:=90.0`

### 2. 左右目顺序反了
- **症状**: stereonet 报 "top is not left image"
- **根因**: mipi_cam 默认 channel=2,0，但左目实际在 channel 0
- **修复**: 改为 `mipi_channel:=0 mipi_channel2:=2`

### 3. camera_info 尺寸不匹配
- **症状**: stereonet 报 "Haven't received any camera info"
- **根因**: mipi_cam 发布 camera_info 尺寸为 sensor 原生 1280×1088，但 GDC 输出 816×960
- **修复**: 使用 640×352 输入尺寸时 stereonet 直接接受原始 camera_info（内部自动处理）

### 4. 1088×1280 原生分辨率启动失败
- **症状**: mipi_cam 报 "cap capture init failure"
- **根因**: 关闭 GDC 用原生竖屏分辨率时驱动初始化失败
- **结论**: GS130W 必须开启 GDC，使用 816×960 或 640×352

### 5. V2.5_int16 帧率瓶颈
- **症状**: 帧率只有 15.7fps，延迟 ~200ms
- **根因**: V2.5_int16 推理上限 16fps
- **修复**: 换 V2.4_int8，帧率提升到 21.3fps

---

## 启动命令

```bash
# 一键启动 (推荐)
source /opt/tros/humble/setup.bash
ros2 launch tracked_vehicle stereo_vision.launch.py

# 或手动分步启动
source /opt/tros/humble/setup.bash

# 1. 双目采集
ros2 launch mipi_cam mipi_cam_dual_channel.launch.py \
  mipi_image_width:=640 mipi_image_height:=352 \
  mipi_image_framerate:=30.0 mipi_out_format:=nv12 \
  mipi_lpwm_enable:=True mipi_frame_ts_type:=realtime \
  mipi_gdc_enable:=True mipi_channel:=0 mipi_channel2:=2 \
  mipi_rotation:=90.0 mipi_cal_rotation:=90.0 log_level:=warn

# 2. 深度估计 + Web 可视化
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int8.launch.py \
  use_mipi_cam:=False \
  stereo_image_topic:=/image_combine_raw \
  camera_info_topic:=/image_combine_raw/right/camera_info \
  left_camera_info_topic:=/image_combine_raw/left/camera_info \
  stereonet_pub_web:=True render_type:=distance \
  render_max_disp:=80 render_z_range:=3.0 infer_thread_num:=2
```

### Web 预览

启动后浏览器访问 `http://<RDK_IP>:8000`

---

## 待优化项

- [ ] 真实双目标定 (当前使用默认内参，畸变全零)
- [ ] 尝试 V2.3 模型 (27fps 上限，延迟可压到 ~120ms)
- [ ] 深度图质量评估 (与真实距离对比)
- [ ] 与 YOLO 人体检测融合 (方案 B: 检测框内深度采点)
