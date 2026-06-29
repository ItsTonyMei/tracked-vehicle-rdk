#!/usr/bin/env python3
"""
voice_bridge — CI1302 语音识别 → /cmd_vel 仲裁节点

职责:
  1. 解析 CI1302 UART 语音识别结果 (AA 55 00 [CMD_ID] FB @ 115200)
  2. 状态机仲裁: VOICE_MANUAL (默认) / FOLLOWING (中继 body_tracking)
  3. 作为 /cmd_vel 唯一发布者，消除多写冲突

状态机:
  VOICE_MANUAL → 语音运动命令直接发布 /cmd_vel，3s 超时自动 STOP
  FOLLOWING   → 中继 /cmd_vel_body_track，语音运动命令暂停 3s 后恢复
                "停止"/"关闭跟随" → VOICE_MANUAL

topic 拓扑:
  voice_bridge 订阅 /cmd_vel_body_track (body_tracking 重映射)
  voice_bridge 发布   /cmd_vel (唯一发布者)
  voice_bridge 发布   /follow_active (Bool, VOICE_MANUAL=False)

协议: AA 55 [STATUS] [CMD_ID] FB
  STATUS=0x00 识别结果, FF=播报触发
  来源: 产品级串口协议列表V3 + 实测验证
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import time
import enum


class State(enum.IntEnum):
    VOICE_MANUAL = 0
    FOLLOWING = 1


class VoiceBridge(Node):

    # 语音命令 → Twist 速度映射
    # IDs 0-9 来自实测验证 (可能与出厂 V3 自定义固件对应)
    # IDs 1-3 来自出厂 V3 标准协议 (0x01/0x02=停止, 0x03=前进)
    # IDs 27-28 来自出厂 V3 标准协议 (0x1B/0x1C=跟随开关)
    CMD_MAP = {
        0:  ('STOP',        (0.0,  0.0,  0.0)),
        1:  ('STOP',        (0.0,  0.0,  0.0)),   # 出厂 V3: 小车停止
        2:  ('STOP',        (0.0,  0.0,  0.0)),   # 出厂 V3: 停止
        3:  ('FORWARD',     (0.5,  0.0,  0.0)),   # 出厂 V3: 小车前进
        4:  ('FORWARD',     (0.5,  0.0,  0.0)),
        5:  ('BACKWARD',    (-0.3,  0.0,  0.0)),
        6:  ('TURN_LEFT',   (0.2,  0.0,  0.4)),
        7:  ('TURN_RIGHT',  (0.2,  0.0, -0.4)),
        8:  ('SPIN_LEFT',   (0.0,  0.0,  0.5)),
        9:  ('SPIN_RIGHT',  (0.0,  0.0, -0.5)),
        27: ('FOLLOW_ON',   None),
        28: ('FOLLOW_OFF',  None),
    }

    _STOP_VEL = (0.0, 0.0, 0.0)

    def __init__(self):
        super().__init__('voice_bridge')

        port = self.declare_parameter('voice_port', '/dev/voice_module').value
        baud = self.declare_parameter('voice_baud', 115200).value
        self._action_duration = self.declare_parameter('action_duration_s', 3.0).value

        # ── 状态机 ──
        self._state = State.VOICE_MANUAL
        self._last_cmd_ts = 0.0
        self._last_cmd_id = None
        self._body_track_msg = None   # 缓存最近一条 body_tracking cmd_vel

        # ── /cmd_vel 唯一发布者 ──
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── /follow_active 状态发布 (供 display_node 显示) ──
        self._follow_pub = self.create_publisher(Bool, '/follow_active', 10)

        # ── 订阅 body_tracking 重映射后的 topic ──
        self._sub_bt = self.create_subscription(
            Twist, '/cmd_vel_body_track', self._on_body_track, 10)

        # ── 串口 ──
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Voice module opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open voice port {port}: {e}')
            raise

        time.sleep(0.5)
        self._ser.flushInput()    # 清空上电噪声，再发 init
        self._write_cmd(0x67)
        self.get_logger().info('Voice bridge ready — VOICE_MANUAL mode')

        self._timer = self.create_timer(0.1, self._poll)
        self._follow_pub.publish(Bool(data=False))

    # ═════════════════════════════════════════════════════════════
    # 生命周期
    # ═════════════════════════════════════════════════════════════

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()

    # ═════════════════════════════════════════════════════════════
    # 串口
    # ═════════════════════════════════════════════════════════════

    def _write_cmd(self, cmd_id):
        frame = bytes([0xAA, 0x55, 0xFF, cmd_id, 0xFB])
        try:
            self._ser.write(frame)
        except serial.SerialException:
            pass

    def _close_serial(self):
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()

    # ═════════════════════════════════════════════════════════════
    # body_tracking 中继
    # ═════════════════════════════════════════════════════════════

    def _on_body_track(self, msg: Twist):
        self._body_track_msg = msg
        # FOLLOWING 且无语音手动介入时直接中继
        if self._state == State.FOLLOWING and self._last_cmd_id is None:
            self._pub.publish(msg)

    # ═════════════════════════════════════════════════════════════
    # 轮询
    # ═════════════════════════════════════════════════════════════

    def _poll(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # ── 运动命令超时处理 ──
        if self._last_cmd_id is not None and now - self._last_cmd_ts > self._action_duration:
            if self._state == State.FOLLOWING:
                # 跟随模式下语音运动暂停结束 → 恢复中继
                self.get_logger().info('Voice motion done, resuming follow relay')
                if self._body_track_msg is not None:
                    self._pub.publish(self._body_track_msg)
            else:
                self._publish_vel('AUTO_STOP', self._STOP_VEL)
            self._last_cmd_id = None

        # ── 读取语音模块 ──
        try:
            count = self._ser.in_waiting
            if not count:
                return
            data = self._ser.read(count)
            # 逐帧解析: AA 55 [STATUS] [CMD_ID] FB
            # 只处理 STATUS=0x00 的识别结果, 忽略唤酾/休眠事件 (0x01/0x02/0x03)
            for i in range(len(data) - 4):
                if data[i] == 0xAA and data[i+1] == 0x55 and data[i+4] == 0xFB:
                    if data[i+2] == 0x00:  # 仅处理识别结果帧
                        self._on_voice(data[i+3])
        except serial.SerialException:
            pass

    # ═════════════════════════════════════════════════════════════
    # 语音命令分发
    # ═════════════════════════════════════════════════════════════

    def _on_voice(self, cmd_id):
        if cmd_id not in self.CMD_MAP:
            self.get_logger().info(f'UNMAPPED voice ID={cmd_id} (0x{cmd_id:02X})')
            return

        name, vel = self.CMD_MAP[cmd_id]
        now = self.get_clock().now().nanoseconds / 1e9

        # ── 模式切换命令 ──
        if cmd_id == 27:  # FOLLOW_ON
            if self._state != State.FOLLOWING:
                self._state = State.FOLLOWING
                self._follow_pub.publish(Bool(data=True))
                self._last_cmd_id = None  # 清除运动中状态
                self.get_logger().info('VOICE: FOLLOW_ON → FOLLOWING mode')
            return

        if cmd_id == 28:  # FOLLOW_OFF
            self._exit_following('VOICE: FOLLOW_OFF')
            return

        # ── 停止命令 ──
        if name == 'STOP':
            if self._state == State.FOLLOWING:
                self._exit_following('VOICE: STOP (exit follow)')
            else:
                self._publish_vel('STOP', self._STOP_VEL)
                self._last_cmd_id = None
            return

        # ── 运动命令 ──
        self._publish_vel(name, vel)
        self._last_cmd_ts = now
        self._last_cmd_id = cmd_id
        self.get_logger().info(
            f'VOICE: {name} (ID={cmd_id}) '
            f'[{self._state.name}]')

    def _exit_following(self, log_msg):
        self._state = State.VOICE_MANUAL
        self._follow_pub.publish(Bool(data=False))
        self._publish_vel('STOP', self._STOP_VEL)
        self._last_cmd_id = None
        self.get_logger().info(f'{log_msg} → VOICE_MANUAL')

    def _publish_vel(self, name, vel):
        msg = Twist()
        msg.linear.x = float(vel[0])
        msg.linear.y = float(vel[1])
        msg.angular.z = float(vel[2])
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = VoiceBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
