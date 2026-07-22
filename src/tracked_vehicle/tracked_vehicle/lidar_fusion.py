#!/usr/bin/env python3
"""
LiDAR-Camera 融合引擎 — 自适应聚类 + 躯干过滤 + 匈牙利匹配 + EKF 状态估计

管线 (以 LiDAR 10Hz 速率驱动):
  /scan (YDLidar T-mini Plus: 360deg, 10Hz, 430pts, 0.84deg/pt)
    -> 自适应距离阈值聚类 (近 0.10m / 远 0.40m)
    -> 躯干几何过滤 (弧宽 15-70cm + 曲率 <0.97, 排除墙壁/柱子)
    -> LiDAR 候选
  camera bboxes (GS130W SC132GS: 72deg HFOV @ 960x544, 1.75mm 广角)
    -> 角度投影 -> Camera 候选
  -> 匈牙利全局最优匹配 (scipy linear_sum_assignment)
  -> EKF(x,y,vx,vy) 预测/更新 -> /locked_target (Point: dist + lateral)

渲染帧 (60Hz) 仅做 EKF predict, 聚类+过滤+匹配+更新仅在 LiDAR 新帧到达时执行.

用法:
  engine = FusionEngine(cam_hfov_deg=72.0)
  for each scan + bboxes:
      result = engine.update(scan_msg, camera_targets, img_w, timestamp)
"""

import math
import numpy as np
from scipy.optimize import linear_sum_assignment


# ═══════════════════════════════════════════════════════════════
# Euclidean 聚类
# ═══════════════════════════════════════════════════════════════

class LidarClusterer:
    """极坐标 → 笛卡尔 → 自适应距离阈值单链聚类.

    距离阈值随 LiDAR 距离动态调整: 近端严格区分腿/杂物, 远端宽松补偿点稀疏."""

    def __init__(self, min_points=3):
        self.min_points = min_points

    @staticmethod
    def _adaptive_thresh(avg_dist):
        """平均距离 → 聚类阈值 (m)."""
        if avg_dist < 1.0:   return 0.10
        if avg_dist < 3.0:   return 0.20
        if avg_dist < 6.0:   return 0.30
        return 0.40

    def cluster(self, ranges, angle_min, angle_inc):
        """返回: [{angle_mid, dist_mean, dist_min, points, angle_span}]
        NaN/Inf 值视为无效点，跳过不参与聚类。"""
        pts_cart = []
        for i, r in enumerate(ranges):
            if not (r > 0.05 and math.isfinite(r)):
                continue
            a = angle_min + i * angle_inc
            pts_cart.append((a, r, math.cos(a) * r, math.sin(a) * r))

        clusters = []
        current = []
        for a, r, x, y in pts_cart:
            if current:
                prev_a, prev_r, px, py = current[-1]
                gap = math.sqrt((x - px)**2 + (y - py)**2)
                if gap > self._adaptive_thresh((r + prev_r) / 2.0):
                    if len(current) >= self.min_points:
                        clusters.append(self._summarize(current))
                    current = []
            current.append((a, r, x, y))
        if len(current) >= self.min_points:
            clusters.append(self._summarize(current))
        return clusters

    @staticmethod
    def _summarize(pts):
        xs = [p[2] for p in pts]
        ys = [p[3] for p in pts]
        cx, cy = float(np.mean(xs)), float(np.mean(ys))
        span = abs(pts[-1][0] - pts[0][0])
        dist_mean = math.hypot(cx, cy)
        width = span * dist_mean  # 弧宽 (m), 小角度近似
        # 曲率: 弦长/弧长, 平坦墙壁 ~1.0, 弧形躯干 <0.95
        chord = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        path = sum(math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(len(xs)-1))
        linearity = chord / path if path > 0.01 else 1.0
        return {
            'angle_mid': (pts[0][0] + pts[-1][0]) / 2.0,
            'angle_span': span,
            'dist_mean': dist_mean,
            'dist_min': float(np.min([p[1] for p in pts])),
            'points': len(pts),
            'width': width,
            'linearity': linearity,
        }

    @staticmethod
    def is_torso(c):
        """躯干几何过滤: 排除墙壁(太平/太宽)和柱子(太窄).

        胸高度 LiDAR 扫描人体躯干: 弧宽 15-70cm, 曲率明显 (linearity < 0.97)."""
        if c['points'] < 3:
            return False
        w = c['width']
        if w < 0.15 or w > 0.70:
            return False
        if c['points'] >= 5 and c['linearity'] > 0.97:
            return False  # 近乎平坦 → 墙壁
        return True


# ═══════════════════════════════════════════════════════════════
# 数据关联 (角度 + 对数距离代价)
# ═══════════════════════════════════════════════════════════════

def _hungarian_match(cost_matrix, max_cost=1.0):
    """匈牙利算法全局最优匹配 (scipy). 返回 [(cam_idx, lid_idx), ...]."""
    n_cam, n_lid = cost_matrix.shape
    if n_cam == 0 or n_lid == 0:
        return []
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    pairs = []
    for r, c in zip(row_ind, col_ind):
        if cost_matrix[r, c] < max_cost:
            pairs.append((int(r), int(c)))
    return pairs


def match_camera_to_lidar(cam_targets, lidar_clusters, cam_hfov_deg=70.0):
    """匹配: 相机 bbox (角度) ↔ LiDAR 聚类 (角度区间)。

    纯角度匹配 + FOV 门控。不依赖 bbox 距离估计 (rotation=90 时不可靠)。
    70° 窄 FOV 内同角度多目标场景极少, 角度匹配已足够。

    cam_targets: [{track_id, angle_deg, angle_span_deg}]
    lidar_clusters: [{angle_mid (rad), angle_span (rad), dist_mean, dist_min}]
    返回: [(cam_idx, lid_idx), ...]"""
    if not cam_targets or not lidar_clusters:
        return []

    half_fov = cam_hfov_deg / 2.0 + 10.0  # +10° 余量容纳 bbox 边缘

    n_cam, n_lid = len(cam_targets), len(lidar_clusters)
    cost = np.full((n_cam, n_lid), 999.0)
    for i, ct in enumerate(cam_targets):
        ca = ct['angle_deg']
        cs = ct['angle_span_deg']
        for j, lc in enumerate(lidar_clusters):
            la = math.degrees(lc['angle_mid'])
            ls = math.degrees(lc['angle_span'])

            # FOV 门控: 雷达聚类必须在相机视野范围内
            if abs(la) > half_fov:
                continue

            # 角度差 (无环绕: 相机 ±35° 内不存在 ±179° 歧义)
            angle_diff = abs(ca - la)

            # 张角重叠惩罚: 零重叠 → span_penalty=1.0 → 需要 angle_diff<90° 才能 <2.0
            overlap = max(0, min(ca + cs/2, la + ls/2) - max(ca - cs/2, la - ls/2))
            span_penalty = 1.0 - overlap / max(cs, ls, 1)
            cost[i, j] = angle_diff / 90.0 + span_penalty

    return _hungarian_match(cost, max_cost=1.0)


# ═══════════════════════════════════════════════════════════════
# 2D EKF — 常速度模型 (Joseph 形式协方差更新)
# ═══════════════════════════════════════════════════════════════

class TargetEKF:
    """状态 [x, y, vx, vy]ᵀ, 观测 [x, y] (从 LiDAR 极坐标转换)。

    R=diag(0.001) 对应 σ≈3.2cm, 匹配 T-mini Plus 近距离实际精度 (~2cm)
    并留有余量覆盖笛卡尔转换噪声。"""

    def __init__(self, x, y, timestamp):
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4) * 0.1
        self.last_t = timestamp
        self.stale = 0
        self.Q = np.diag([0.5, 0.5, 2.0, 2.0])  # ×dt 缩放后等效连续谱密度
        self.R = np.diag([0.001, 0.001])      # σ ≈ 3.2cm (was 0.04=20cm)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float64)
        self.I = np.eye(4)

    def predict(self, t):
        dt = t - self.last_t
        if dt <= 0:
            return
        self.last_t = t
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1,  0],
                      [0, 0, 0,  1]], dtype=np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, z_x, z_y):
        z = np.array([z_x, z_y])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        # Joseph 形式: 数值更稳定, 保证 P 正定
        I_KH = self.I - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        self.stale = 0

    @property
    def state(self):
        return {'x': float(self.x[0]), 'y': float(self.x[1]),
                'vx': float(self.x[2]), 'vy': float(self.x[3]),
                'speed': math.hypot(self.x[2], self.x[3]),
                'stale': self.stale}


# ═══════════════════════════════════════════════════════════════
# 融合引擎
# ═══════════════════════════════════════════════════════════════

def _find_body_roi(target):
    for roi in target.rois:
        if roi.type == 'body':
            return roi.rect
    return None


class FusionEngine:
    """顶层融合管线: cluster → match → EKF predict → EKF update → prune stale.

    速率解耦: predict() 每次调用都执行 (适配渲染 60Hz);
    cluster+match+EKF update 仅在新 LiDAR 帧到达时执行 (10Hz).
    stale 计数器因此绑定 LiDAR 帧率, ekf_max_stale=5 正确对应 0.5s.

    Track 管理策略:
      - 人物 track (tid >= 0): 不设数量上限, 由 staleness 自然淘汰.
      - 障碍物 track (tid < 0): 上限 MAX_OBS_TRACKS, 满时驱逐最旧的.
    """

    MAX_OBS_TRACKS = 15

    PERSON_STALE_MAX = 15   # 人物: 1.5s 超时 (was 30=3s, 更快恢复异常 track)
    OBS_STALE_MAX    = 5    # 障碍物: 0.5s 超时 (快速清理)

    def __init__(self, cam_hfov_deg=72.0):
        self.clusterer = LidarClusterer()
        self.cam_hfov = cam_hfov_deg
        self._tracks = {}                  # track_id → EKF (Python 3.7+ ordered)
        self._next_obs_id = 1000
        self._scan_ranges = None
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0
        self._needs_update = False         # 新扫描到达标志
        self._cached_result = {}           # 渲染帧间复用结果
        self._scan_msg_stamp = 0.0         # 跟踪扫描时间戳避免重复处理

    def update(self, scan_msg, camera_targets, img_w, timestamp):
        """每帧调用 (60Hz 渲染). 仅在新 LiDAR 帧到达时执行完整管线."""
        # ── 缓存新 scan, 标记需要更新 ──
        if scan_msg is not None:
            stamp = getattr(scan_msg.header, 'stamp', None)
            stamp_s = stamp.sec + stamp.nanosec * 1e-9 if stamp else timestamp
            if stamp_s != self._scan_msg_stamp:
                self._scan_ranges = list(scan_msg.ranges)
                self._scan_angle_min = scan_msg.angle_min
                self._scan_angle_inc = scan_msg.angle_increment
                self._scan_msg_stamp = stamp_s
                self._needs_update = True

        # ── Predict 全部已有 track (轻量, 60Hz 执行) ──
        for tid in self._tracks:
            self._tracks[tid].predict(timestamp)

        # ── 非 LiDAR 更新帧 → 仅 predict, 返回缓存结果 ──
        if not self._needs_update:
            return self._cached_result

        self._needs_update = False

        if self._scan_ranges is None or len(self._scan_ranges) == 0:
            self._cached_result = {}
            return {}

        # ── 1. LiDAR 聚类 (仅新扫描帧) ──
        clusters = self.clusterer.cluster(
            self._scan_ranges, self._scan_angle_min, self._scan_angle_inc)
        # 躯干过滤: 仅对匹配池过滤, 障碍物保留完整聚类
        torso_clusters = [c for c in clusters if LidarClusterer.is_torso(c)]

        # ── 2. 相机 bbox → 角度投影 ──
        cam_list = []
        if camera_targets is not None:
            for t in camera_targets.targets:
                body_roi = _find_body_roi(t)
                if body_roi is None:
                    continue
                r = body_roi
                cx = r.x_offset + r.width / 2.0
                angle_deg = (cx / img_w - 0.5) * self.cam_hfov
                angle_span_deg = (r.width / img_w) * self.cam_hfov
                cam_list.append({
                    'track_id': t.track_id,
                    'angle_deg': angle_deg,
                    'angle_span_deg': angle_span_deg,
                })

        # ── 3. 数据关联 (角度+距离代价 + FOV 门控, 仅躯干聚类) ──
        pairs = match_camera_to_lidar(cam_list, torso_clusters, self.cam_hfov)
        # 反查: torso 索引 → 原始 cluster 角度 (用于障碍物去重)
        matched_angles = set(torso_clusters[l]['angle_mid'] for _, l in pairs)

        # ── 4. EKF 更新匹配对 ──
        # 统一递增 stale: 匹配成功的由 update() 重置为 0
        for tid in self._tracks:
            self._tracks[tid].stale += 1

        MAX_DIST_JUMP = 0.5  # 米, 0.1s 内人最多移动 ~0.2m; 0.5m 过滤误匹配
        for ci, lj in pairs:
            ct = cam_list[ci]
            lc = torso_clusters[lj]
            tid = ct['track_id']
            a_rad = lc['angle_mid']
            d = lc['dist_mean']
            lx = math.cos(a_rad) * d
            ly = math.sin(a_rad) * d

            if tid not in self._tracks:
                ekf = TargetEKF(lx, ly, timestamp)
                self._tracks[tid] = ekf
            else:
                ekf = self._tracks[tid]
                cur = ekf.state
                jump = math.hypot(lx - cur['x'], ly - cur['y'])
                if jump < MAX_DIST_JUMP:
                    ekf.update(lx, ly)      # update() 内部重置 stale=0

        # ── 5. 未匹配的 LiDAR 聚类 → 纯障碍物 (角度匹配持久 ID) ──
        for j, lc in enumerate(clusters):
            if lc['angle_mid'] in matched_angles:
                continue
            a_rad = lc['angle_mid']
            d = lc['dist_mean']
            lx = math.cos(a_rad) * d
            ly = math.sin(a_rad) * d

            # 角度匹配: 在现有障碍物 track 中找最近角度 (< 5°)
            best_obs_id, best_obs_angle = None, float('inf')
            for oid, ekf in self._tracks.items():
                if oid >= 0:
                    continue
                oa = math.atan2(ekf.x[1], ekf.x[0])
                angle_diff = abs(a_rad - oa)
                if angle_diff < math.radians(5) and angle_diff < best_obs_angle:
                    best_obs_id, best_obs_angle = oid, angle_diff

            if best_obs_id is not None:
                obs_id = best_obs_id
                ekf = self._tracks[obs_id]
                cur = ekf.state
                jump = math.hypot(lx - cur['x'], ly - cur['y'])
                if jump < MAX_DIST_JUMP:
                    ekf.update(lx, ly)
            else:
                obs_tracks = [oid for oid in self._tracks if oid < 0]
                if len(obs_tracks) >= self.MAX_OBS_TRACKS:
                    oldest = max(obs_tracks, key=lambda oid: self._tracks[oid].stale)
                    del self._tracks[oldest]
                obs_id = -self._next_obs_id
                self._next_obs_id += 1
                ekf = TargetEKF(lx, ly, timestamp)
                self._tracks[obs_id] = ekf

        # ── 6. 清理过期 track (人物/障碍物不同超时) ──
        stale_ids = []
        for tid, ekf in self._tracks.items():
            limit = self.PERSON_STALE_MAX if tid >= 0 else self.OBS_STALE_MAX
            if ekf.stale > limit:
                stale_ids.append(tid)
        for tid in stale_ids:
            del self._tracks[tid]

        # ── 7. 构建输出 ──
        result = {}
        for tid, ekf in self._tracks.items():
            s = ekf.state
            result[tid] = {'dist': math.hypot(s['x'], s['y']),
                          'x': s['x'], 'y': s['y']}

        self._cached_result = result
        return result
