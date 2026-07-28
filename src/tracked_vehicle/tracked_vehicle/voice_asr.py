#!/usr/bin/env python3
"""voice_asr — 离线中文语音识别节点 (Vosk)

状态机:
  SLEEP  ──唤醒词"你好瓦力"──→  AWAKE
  AWAKE  ──识别到命令────────→  发布 /voice_cmd  →  SLEEP
  AWAKE  ──8s 无有效命令─────→  SLEEP

命令映射复用 motion_arbiter CMD_MAP 的 ID 体系.

硬件: M30 USB 麦克风, 16kHz 单声道, arecord 管道直读
模型: Vosk small-cn-0.22 (~66MB), /home/sunrise/tts_models/vosk-cn
"""

import os
import json
import logging
import subprocess
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

_log = logging.getLogger('voice_asr')

# ── 唤醒词 + 命令词映射 ──────────────────────────────

WAKE_WORDS = ['你好瓦力', '瓦力', '瓦砾', 'hello 瓦力', '你好瓦砾']

COMMAND_MAP = {
    0x06: ['停车', '停止', '停下', '站住'],
    0x07: ['前进', '向前', '往前走', '直走'],
    0x08: ['后退', '向后', '往后', '倒退'],
    0x09: ['左转', '向左转', '往左转'],
    0x0A: ['右转', '向右转', '往右转'],
    0x0B: ['左旋', '向左旋', '原地左转'],
    0x0C: ['右旋', '向右旋', '原地右转'],
    0x0D: ['跟我走', '跟随', '开启跟随', '跟着我'],
    0x0E: ['别跟我', '关闭跟随', '停止跟随', '别跟着'],
}

_KEYWORD_TO_CMD = {}
for cmd_id, keywords in COMMAND_MAP.items():
    for kw in keywords:
        _KEYWORD_TO_CMD[kw] = cmd_id


class VoiceAsr(Node):
    """离线语音识别节点."""

    def __init__(self):
        super().__init__('voice_asr')

        model_dir = self.declare_parameter(
            'vosk_model', '/home/sunrise/tts_models/vosk-cn').value
        mic_device = self.declare_parameter(
            'mic_device', 'plughw:0,0').value
        self._wake_timeout = self.declare_parameter(
            'wake_timeout_s', 8.0).value
        self._mic_gain = self.declare_parameter(
            'mic_gain', 10.0).value  # M30 麦克风音量偏低, 软件补偿

        # ── 加载 Vosk 模型 ──
        try:
            import vosk
            if not os.path.isdir(model_dir):
                self.get_logger().fatal(f'Vosk model not found: {model_dir}')
                raise FileNotFoundError(model_dir)
            self._model = vosk.Model(model_dir)
            self._recognizer = vosk.KaldiRecognizer(self._model, 16000)
            self.get_logger().info(f'Vosk model loaded: {model_dir}')
        except Exception as e:
            self.get_logger().fatal(f'Vosk init failed: {e}')
            raise

        # ── 状态机 ──
        self._state = 'SLEEP'
        self._awake_since = 0.0

        # ── 发布 ──
        self._cmd_pub = self.create_publisher(Int32, '/voice_cmd', 10)
        self._partial_pub = self.create_publisher(String, '/voice_partial', 10)

        # ── 启动 arecord 进程 (16kHz mono S16_LE → stdout) ──
        self._arecord = subprocess.Popen(
            ['arecord', '-q', '-D', mic_device,
             '-f', 'S16_LE', '-r', '16000', '-c', '1'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.get_logger().info(
            f'arecord started: device={mic_device}, pid={self._arecord.pid}')

        # ── 音频读取线程 ──
        self._audio_buffer = b''
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(
            target=self._read_audio, daemon=True)
        self._reader_thread.start()

        # ── 主循环定时器 (100ms) ──
        self._timer = self.create_timer(0.1, self._process_audio)
        self.get_logger().info(
            f'Voice ASR ready — state={self._state}, mic={mic_device}')

    def _read_audio(self):
        """后台线程: 从 arecord stdout 持续读取音频, 应用软件增益."""
        chunk_size = 3200  # 100ms @ 16kHz S16_LE mono = 3200 bytes
        while not self._stop_event.is_set():
            try:
                data = self._arecord.stdout.read(chunk_size)
                if not data:
                    break
                if self._mic_gain != 1.0:
                    arr = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                    arr *= self._mic_gain
                    np.clip(arr, -32768, 32767, out=arr)
                    data = arr.astype(np.int16).tobytes()
                self._audio_buffer += data
            except Exception:
                break

    def _process_audio(self):
        """主循环: 将缓冲音频送给 Vosk, 执行状态机."""
        now = self.get_clock().now().nanoseconds / 1e9

        if len(self._audio_buffer) < 800:  # < 25ms, 等积累
            return

        # 每次取 100ms (~3200 bytes) 送识别器
        chunk = self._audio_buffer[:3200]
        self._audio_buffer = self._audio_buffer[3200:]

        if self._recognizer.AcceptWaveform(chunk):
            result = json.loads(self._recognizer.Result())
            text = result.get('text', '').strip()
            if text:
                self.get_logger().info(f'Final: "{text}"')
                self._on_recognized(text, now)
        else:
            partial = json.loads(self._recognizer.PartialResult())
            ptext = partial.get('partial', '').strip()
            if ptext:
                self._partial_pub.publish(String(data=ptext))

    # ── 状态机 ──────────────────────────────────────────

    def _on_recognized(self, text: str, now: float):
        text_lower = text.replace(' ', '')
        if self._state == 'SLEEP':
            self._handle_sleep(text_lower, now)
        elif self._state == 'AWAKE':
            self._handle_awake(text_lower, now)

    def _handle_sleep(self, text: str, now: float):
        for wake_word in WAKE_WORDS:
            if wake_word in text:
                self._state = 'AWAKE'
                self._awake_since = now
                self.get_logger().info(f'WAKE: "{wake_word}" → AWAKE')
                return

    def _handle_awake(self, text: str, now: float):
        if now - self._awake_since > self._wake_timeout:
            self.get_logger().info('AWAKE timeout → SLEEP')
            self._state = 'SLEEP'
            return

        for keyword, cid in _KEYWORD_TO_CMD.items():
            if keyword in text:
                self.get_logger().info(
                    f'CMD: "{text}" → {keyword} (0x{cid:02X})')
                self._cmd_pub.publish(Int32(data=cid))
                self._state = 'SLEEP'
                return

        self.get_logger().info(f'No match: "{text}" → SLEEP')
        self._state = 'SLEEP'

    def destroy_node(self):
        self._stop_event.set()
        try:
            self._arecord.terminate()
            self._arecord.wait(timeout=2)
        except Exception:
            self._arecord.kill()
        self._reader_thread.join(timeout=1)
        super().destroy_node()


def main():
    rclpy.init()
    node = VoiceAsr()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
