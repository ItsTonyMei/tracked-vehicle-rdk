#!/usr/bin/env python3
"""
LiDAR-Camera 融合引擎 — 聚类 + 数据关联 + EKF 状态估计

管线:
  /scan (430pts, 10Hz) → Euclidean 聚类 → LiDAR 目标
  camera bboxes         → 角度投影      → Camera 目标
  ── Hungarian 匹配 ──→ EKF 更新/预测 ──→ 融合距离 + 速度

用法:
  engine = FusionEngine()
  for each scan + bboxes:
      result = engine.update(scan_msg, camera_bboxes, timestamp)
      # result[track_id] = {'dist': 1.85, 'x': 1.84, 'y': -0.15, 'vx': 0.3, 'vy': 0.0}
"""

import math
import numpy as np
from collections import OrderedDict


# ═══════════════════════════════════════════════════════════════
# Euclidean 聚类
# ═══════════════════════════════════════════════════════════════

class LidarClusterer:
    """极坐标 → 笛卡尔 → 距离聚类 (DBSCAN 简化变体)。"""

    def __init__(self, dist_thresh=0.20, min_points=3):
        self.dist_thresh = dist_thresh   # 相邻两点聚类阈值 (m)
        self.min_points = min_points      # 最小点数

    def cluster(self, ranges, angle_min, angle_inc):
        """返回: [{angle_mid, dist_mean, dist_min, points, angle_span}]"""
        pts_cart = []
        for i, r in enumerate(ranges):
            a = angle_min + i * angle_inc
            pts_cart.append((a, r, math.cos(a) * r, math.sin(a) * r))

        clusters = []
        current = []
        for a, r, x, y in pts_cart:
            if current:
                prev_a, prev_r, px, py = current[-1]
                gap = math.sqrt((x - px)**2 + (y - py)**2)
                if gap > self.dist_thresh:
                    if len(current) >= self.min_points:
                        clusters.append(self._summarize(current))
                    current = []
            current.append((a, r, x, y))
        if len(current) >= self.min_points:
            clusters.append(self._summarize(current))
        return clusters

    @staticmethod
    def _summarize(pts):
        angles = [p[0] for p in pts]
        ranges = [p[1] for p in pts]
        valid = [r for r in ranges if r > 0.05]
        if not valid:
            valid = ranges
        return {
            'angle_mid': (angles[0] + angles[-1]) / 2.0,
            'angle_span': abs(angles[-1] - angles[0]),
            'dist_mean': float(np.mean(valid)),
            'dist_min': float(np.min(valid)),
            'points': len(pts),
        }


# ═══════════════════════════════════════════════════════════════
# 匈牙利数据关联
# ═══════════════════════════════════════════════════════════════

def _hungarian(cost_matrix):
    """穷举匈牙利分配 (n ≤ 10, 无需 scipy)。返回 [(cam_idx, lid_idx), ...]."""
    n_cam, n_lid = cost_matrix.shape
    if n_cam == 0 or n_lid == 0:
        return []
    # 贪心 + 最小化: 对每个 cam 找最佳 lid
    used_lid = set()
    pairs = []
    cam_order = sorted(range(n_cam), key=lambda i: np.min(cost_matrix[i]))
    for ci in cam_order:
        best_j, best_c = -1, float('inf')
        for j in range(n_lid):
            if j not in used_lid and cost_matrix[ci, j] < best_c:
                best_c, best_j = cost_matrix[ci, j], j
        if best_j >= 0 and best_c < 1.0:
            pairs.append((ci, best_j))
            used_lid.add(best_j)
    return pairs


def match_camera_to_lidar(cam_targets, lidar_clusters):
    """匹配: 相机 bbox (角度) ↔ LiDAR 聚类 (角度区间)。
    cam_targets: [{track_id, angle_deg, angle_span_deg, bbox_dist}]
    lidar_clusters: [{angle_mid (rad), angle_span (rad), dist_mean, dist_min}]
    返回: [(cam_idx, lid_idx, cost), ...]"""
    if not cam_targets or not lidar_clusters:
        return []
    n_cam, n_lid = len(cam_targets), len(lidar_clusters)
    cost = np.full((n_cam, n_lid), 999.0)
    for i, ct in enumerate(cam_targets):
        ca = ct['angle_deg']
        cs = ct['angle_span_deg']
        for j, lc in enumerate(lidar_clusters):
            la = math.degrees(lc['angle_mid'])
            ls = math.degrees(lc['angle_span'])
            # 角度中心差 + 张角 IoU
            angle_diff = abs(ca - la)
            # 归一化到 [0, 180] 处理角度环绕
            angle_diff = min(angle_diff, 360 - angle_diff)
            # 张角重叠惩罚
            overlap = max(0, min(ca + cs/2, la + ls/2) - max(ca - cs/2, la - ls/2))
            span_penalty = 1.0 - overlap / max(cs, ls, 1)
            cost[i, j] = angle_diff / 90.0 + span_penalty
    return _hungarian(cost)


# ═══════════════════════════════════════════════════════════════
# 2D EKF — 常速度模型
# ═══════════════════════════════════════════════════════════════

class TargetEKF:
    """状态 [x, y, vx, vy]ᵀ, 观测 [x, y] (从 LiDAR 极坐标转换)。"""

    def __init__(self, x, y, timestamp):
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4) * 0.5
        self.last_t = timestamp
        self.stale = 0
        self.Q = np.diag([0.05, 0.05, 0.2, 0.2])
        self.R = np.diag([0.04, 0.04])
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float64)
        self.I = np.eye(4)

    def predict(self, t):
        dt = t - self.last_t
        if dt <= 0:
            return
        self.last_t = t
        # F 矩阵
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1,  0],
                      [0, 0, 0,  1]], dtype=np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        self.stale += 1

    def update(self, z_x, z_y):
        z = np.array([z_x, z_y])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P
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

class FusionEngine:
    """顶层融合管线: cluster → match → EKF predict → EKF update → prune stale."""

    def __init__(self, cam_hfov_deg=70.0, cluster_dist=0.20,
                 ekf_max_stale=15, max_targets=8):
        self.clusterer = LidarClusterer(dist_thresh=cluster_dist)
        self.cam_hfov = cam_hfov_deg
        self.ekf_max_stale = ekf_max_stale
        self.max_targets = max_targets
        self._tracks = OrderedDict()       # track_id → EKF
        self._last_scan_t = 0.0
        self._scan_ranges = None
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0

    def update(self, scan_msg, camera_targets, img_w, timestamp):
        """每帧调用: 传入 /scan 消息 + camera bbox 列表.
        camera_targets: PerceptionTargets.targets 列表
        返回: {track_id: {dist, x, y, vx, vy, source}}"""
        # ── 缓存 scan 数据 ──
        if scan_msg is not None:
            self._scan_ranges = list(scan_msg.ranges)
            self._scan_angle_min = scan_msg.angle_min
            self._scan_angle_inc = scan_msg.angle_increment
            self._last_scan_t = timestamp

        if self._scan_ranges is None or len(self._scan_ranges) == 0:
            return {}

        # ── 1. LiDAR 聚类 ──
        clusters = self.clusterer.cluster(
            self._scan_ranges, self._scan_angle_min, self._scan_angle_inc)

        # ── 2. 相机 bbox → 角度投影 ──
        cam_list = []
        if camera_targets is not None:
            for t in camera_targets.targets:
                body_roi = self._find_body_roi(t)
                if body_roi is None:
                    continue
                r = body_roi
                cx = r.x_offset + r.width / 2.0
                # 像素坐标 → 角度
                angle_deg = (cx / img_w - 0.5) * self.cam_hfov
                # bbox 宽度 → 角度张角
                angle_span_deg = (r.width / img_w) * self.cam_hfov
                # 备用: bbox 宽度估计距离
                est_dist = 1000.0 / r.width if r.width > 0 else 0
                cam_list.append({
                    'track_id': t.track_id,
                    'angle_deg': angle_deg,
                    'angle_span_deg': angle_span_deg,
                    'bbox_dist': est_dist,
                })

        # ── 3. 数据关联 ──
        pairs = match_camera_to_lidar(cam_list, clusters)
        matched_cams = set(c for c, _ in pairs)
        matched_lids = set(l for _, l in pairs)

        # ── 4. EKF 预测全部已有 track ──
        for tid in self._tracks:
            self._tracks[tid].predict(timestamp)

        # ── 5. EKF 更新匹配对 ──
        MAX_DIST_JUMP = 2.0  # 米, 匹配距离跳变超过此值认为是误匹配
        for ci, lj in pairs:
            ct = cam_list[ci]
            lc = clusters[lj]
            tid = ct['track_id']
            # LiDAR 笛卡尔坐标
            a_rad = lc['angle_mid']
            d = lc['dist_mean']
            lx = math.cos(a_rad) * d
            ly = math.sin(a_rad) * d

            if tid not in self._tracks:
                if len(self._tracks) >= self.max_targets:
                    continue
                ekf = TargetEKF(lx, ly, timestamp)
                self._tracks[tid] = ekf
            else:
                ekf = self._tracks[tid]
                cur = ekf.state
                jump = math.hypot(lx - cur['x'], ly - cur['y'])
                if jump < MAX_DIST_JUMP:
                    ekf.update(lx, ly)

        # ── 6. 处理未匹配的 LiDAR 聚类 → 纯障碍物 ──
        for j, lc in enumerate(clusters):
            if j in matched_lids:
                continue
            a_rad = lc['angle_mid']
            d = lc['dist_mean']
            lx = math.cos(a_rad) * d
            ly = math.sin(a_rad) * d
            # 为纯雷达障碍物生成一个负 ID
            obs_id = -1000 - j
            if obs_id not in self._tracks:
                ekf = TargetEKF(lx, ly, timestamp)
                self._tracks[obs_id] = ekf
            else:
                self._tracks[obs_id].update(lx, ly)

        # ── 7. 清理过期 track ──
        stale_ids = [tid for tid, ekf in self._tracks.items()
                     if ekf.state['stale'] > self.ekf_max_stale]
        for tid in stale_ids:
            del self._tracks[tid]

        # ── 8. 构建输出 ──
        result = {}
        for tid, ekf in self._tracks.items():
            s = ekf.state
            dist = math.hypot(s['x'], s['y'])
            source = 'lidar' if tid in matched_cams else 'bbox'
            # 检查 LiDAR 距离与 bbox 估计的一致性
            if tid in matched_cams:
                for ct in cam_list:
                    if ct['track_id'] == tid:
                        bbox_d = ct['bbox_dist']
                        if dist < bbox_d * 0.4:
                            # LiDAR 距离远小于 bbox 估计 → 雷达打到了前景物体而非目标
                            source = 'bbox'
                            dist = bbox_d
                        break

            result[tid] = {'dist': dist, 'x': s['x'], 'y': s['y'],
                          'vx': s['vx'], 'vy': s['vy'],
                          'speed': s['speed'], 'source': source}
        return result

    def get_distance(self, track_id):
        """便捷方法: 返回融合距离 (米) 或 None."""
        if track_id in self._tracks:
            s = self._tracks[track_id].state
            return math.hypot(s['x'], s['y'])
        return None

    @staticmethod
    def _find_body_roi(target):
        for roi in target.rois:
            if roi.type == 'body':
                return roi.rect
        return None
