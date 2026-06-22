#!/usr/bin/env python3
"""本地屏显 — 触摸点选追踪 + 骨骼可视化 + 隐藏光标"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ai_msgs.msg import PerceptionTargets
import cv2
import numpy as np
import os

# COCO 骨骼连接 + 关键点颜色
SKELETON = [(5,6),(5,7),(7,9),(6,8),(8,10),(11,12),
            (11,13),(13,15),(12,14),(14,16),(0,1),(0,2),(1,3),(2,4)]
KP_COLORS = [(255,0,0),(255,85,0),(255,170,0),(255,255,0),
             (170,255,0),(85,255,0),(0,255,0),(0,255,85),
             (0,255,170),(0,255,255),(0,170,255),(0,85,255),
             (0,0,255),(85,0,255),(170,0,255),(255,0,255),(255,0,170),(255,0,85)]


def hide_cursor():
    """隐藏 X11 光标 (保留点击功能)"""
    try:
        # 创建 1x1 透明光标
        os.system('xsetroot -cursor /dev/null 2>/dev/null || true')
        # 备选: 用 X11 创建空光标
        os.popen('python3 -c "'
            'from ctypes import cdll, c_char_p;'
            'x11=cdll.LoadLibrary(\"libX11.so.6\");'
            'd=x11.XOpenDisplay(None);'
            'w=x11.XDefaultRootWindow(d);'
            'bm=x11.XCreateBitmapFromData(d,w,c_char_p(bytes(8)),1,1);'
            'c=x11.XCreatePixmapCursor(d,bm,bm,0,0,0,0);'
            'x11.XDefineCursor(d,w,c);'
            'x11.XFlush(d)" 2>/dev/null || true')
    except Exception:
        pass


class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.target_dist = self.declare_parameter('target_dist', 2.0).value
        self.bbox_ref = self.declare_parameter('bbox_ref_width', 500.0).value
        self.bbox_ref_dist = self.declare_parameter('bbox_ref_dist', 2.0).value
        self.rotate_deg = self.declare_parameter('rotate_deg', 0).value

        self._frame = None
        self._targets = None
        self._selected_id = None   # 被点选的目标 track_id
        self._dbl_click_ts = 0.0   # 双击检测时间戳
        self._window = 'RDK X5 Tracker'
        self._init_display()

    def _init_display(self):
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, 1024, 600)
        cv2.moveWindow(self._window, 0, 0)
        cv2.setWindowProperty(self._window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.setMouseCallback(self._window, self._on_mouse)
        hide_cursor()
        self.get_logger().info('display_node OK (touch select enabled)')

    def img_cb(self, msg: CompressedImage):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        self._frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)

    def det_cb(self, msg: PerceptionTargets):
        self._targets = msg

    def _rotate(self, img):
        if self.rotate_deg == 90:
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_deg in (270, -90):
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_deg == 180:
            return cv2.rotate(img, cv2.ROTATE_180)
        return img

    # ── 坐标变换: 屏幕 → 原图 ─────────────────────────
    def _screen_to_orig(self, sx, sy):
        """将 1024x600 屏幕坐标映射回原始 960x544 图像坐标"""
        scr_w, scr_h = 1024, 600
        fh, fw = self._frame.shape[:2]
        scale = max(scr_w / fw, scr_h / fh)
        nw, nh = int(fw * scale), int(fh * scale)
        # 逆裁切
        sx_full = sx + (nw - scr_w) // 2
        sy_full = sy + (nh - scr_h) // 2
        # 逆缩放
        ox = int(sx_full / scale)
        oy = int(sy_full / scale)
        return ox, oy

    def _find_person_at(self, ox, oy):
        """查找坐标 (原图系) 处的人体框"""
        if self._targets is None:
            return None
        for t in self._targets.targets:
            if t.type != 'person':
                continue
            for roi in t.rois:
                if roi.type != 'body':
                    continue
                r = roi.rect
                if (r.x_offset <= ox <= r.x_offset + r.width and
                    r.y_offset <= oy <= r.y_offset + r.height):
                    return t
        return None

    # ── 鼠标/触摸回调 ─────────────────────────────────
    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            ox, oy = self._screen_to_orig(x, y)
            person = self._find_person_at(ox, oy)
            if person is not None:
                self._selected_id = person.track_id
                self.get_logger().info(f'SELECTED track_id={person.track_id}')
            else:
                self._selected_id = None
                self.get_logger().info('DESELECTED')

    # ── 渲染 ──────────────────────────────────────────
    def render(self):
        if self._frame is None:
            return
        frame = self._frame.copy()
        orig_h, orig_w = frame.shape[:2]

        targets = self._targets
        if targets is not None:
            for t in targets.targets:
                if t.type != 'person':
                    continue
                body_roi = None
                for roi in t.rois:
                    if roi.type == 'body':
                        body_roi = roi.rect
                        break
                if body_roi is None:
                    continue

                r = body_roi
                x1, y1 = int(r.x_offset), int(r.y_offset)
                x2, y2 = x1 + int(r.width), y1 + int(r.height)
                dist = (self.bbox_ref * self.bbox_ref_dist) / r.width if r.width > 0 else 0

                # 框颜色: 选中=红, 未选中=绿
                is_selected = (t.track_id == self._selected_id)
                box_color = (0, 0, 255) if is_selected else (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 3 if is_selected else 2)

                # 骨骼关键点
                kps = None
                for pt in t.points:
                    if pt.type == 'body_kps':
                        kps = [(int(p.x), int(p.y)) for p in pt.point]
                        break

                if kps and len(kps) >= 17:
                    for i, j in SKELETON:
                        if i < len(kps) and j < len(kps):
                            cv2.line(frame, kps[i], kps[j], (0, 255, 255), 1)
                    for idx, (kx, ky) in enumerate(kps[:17]):
                        color = KP_COLORS[min(idx, len(KP_COLORS)-1)]
                        cv2.circle(frame, (kx, ky), 3, color, -1)
                    nose = kps[0]
                    eye_dist = abs(kps[1][0] - kps[2][0]) if len(kps) > 2 else 20
                    head_r = max(int(eye_dist * 0.6), 15)
                    cv2.circle(frame, nose, head_r, (0, 255, 255), 2)
                else:
                    head_roi = None
                    for roi in t.rois:
                        if roi.type == 'head':
                            head_roi = roi.rect
                            break
                    hx1, hy1 = (int(head_roi.x_offset), int(head_roi.y_offset)) if head_roi else (x1, y1)
                    hx2, hy2 = (hx1 + int(head_roi.width), hy1 + int(head_roi.height)) if head_roi else (x2, y1 + (y2-y1)//3)
                    cv2.rectangle(frame, (hx1, hy1), (hx2, hy2), (255, 255, 0), 2)

                # 标签
                prefix = '>> ' if is_selected else ''
                label = f'{prefix}#{t.track_id} {dist:.1f}m'
                cv2.rectangle(frame, (x1, y1 - 24), (x1 + 200, y1), box_color, -1)
                cv2.putText(frame, label, (x1 + 4, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # 偏移线
                bx, by = x1 + int(r.width)//2, y1 + int(r.height)//2
                cv2.line(frame, (bx, by), (orig_w//2, orig_h//2), (255, 0, 255), 1)

        # 中心十字 + 死区
        cx0, cy0 = orig_w // 2, orig_h // 2
        cv2.line(frame, (cx0 - 20, cy0), (cx0 + 20, cy0), (128, 128, 128), 1)
        cv2.line(frame, (cx0, cy0 - 20), (cx0, cy0 + 20), (128, 128, 128), 1)
        cv2.ellipse(frame, (cx0, cy0), (40, 40), 0, 0, 360, (100, 100, 100), 1)

        # 旋转
        if self.rotate_deg:
            frame = self._rotate(frame)

        # 缩放填充全屏
        scr_w, scr_h = 1024, 600
        fh, fw = frame.shape[:2]
        scale = max(scr_w / fw, scr_h / fh)
        nw, nh = int(fw * scale), int(fh * scale)
        frame = cv2.resize(frame, (nw, nh))
        sx, sy = (nw - scr_w)//2, (nh - scr_h)//2
        frame = frame[sy:sy+scr_h, sx:sx+scr_w]
        h, w = frame.shape[:2]

        # 状态条
        if targets and targets.targets:
            t0 = targets.targets[0]
            sel = f' | SELECTED #{self._selected_id}' if self._selected_id else ''
            status = f'TRACK #{t0.track_id}{sel} | {targets.fps}FPS | dbl-click to select'
            color = (0, 255, 0)
        else:
            status = 'NO PERSON | dbl-click to select'
            color = (0, 0, 255)
        cv2.putText(frame, status, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow(self._window, frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = DisplayNode()
    node.sub_img = node.create_subscription(CompressedImage, '/image', node.img_cb, 10)
    node.sub_det = node.create_subscription(PerceptionTargets, '/hobot_mono2d_body_detection', node.det_cb, 10)
    node.timer = node.create_timer(0.05, node.render)
    rclpy.spin(node)
    rclpy.shutdown()
