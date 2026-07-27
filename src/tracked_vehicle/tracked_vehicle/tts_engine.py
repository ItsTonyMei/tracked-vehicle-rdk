#!/usr/bin/env python3
"""TTS 引擎 — 多后端离线中文语音合成, 输出到 M30 USB 扬声器.

后端优先级:
  1. piper-tts   — 神经网络合成, 中文音质好 (~50MB 模型)
  2. espeak-ng   — 规则合成, 零模型下载, 永远可用 (兜底)

单例模式, speak() 非阻塞 (后台线程播放).

配置:
    PIPER_MODEL_DIR — piper 模型目录, 默认 /home/sunrise/tts_models/piper-zh
    TTS_DEVICE      — ALSA 输出设备, 默认 plughw:0,0 (M30 USB)
"""

import os
import logging
import threading
import subprocess
import tempfile

_log = logging.getLogger('tts_engine')


class TTSEngine:
    """单例 TTS 引擎, 惰性初始化."""

    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        self._voice = None        # piper.PiperVoice or None
        self._model_dir = os.environ.get(
            'PIPER_MODEL_DIR',
            '/home/sunrise/tts_models/piper-zh')
        self._device = os.environ.get('TTS_DEVICE', 'plughw:0,0')
        self._backend = 'none'
        self._init_ok = False

    @classmethod
    def get(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    # ── 初始化 ──────────────────────────────────────────

    def _ensure_init(self):
        if self._init_ok:
            return True
        with self._lock:
            if self._init_ok:
                return True

            if self._try_piper():
                self._backend = 'piper'
                self._init_ok = True
                _log.info('TTS ready: piper-tts')
                return True

            if self._try_espeak():
                self._backend = 'espeak'
                self._init_ok = True
                _log.info('TTS ready: espeak-ng (fallback)')
                return True

            _log.error('TTS: no backend available')
            return False

    def _try_piper(self):
        try:
            import piper
            # 扫描模型目录找 .onnx 文件
            candidates = sorted([
                f for f in os.listdir(self._model_dir)
                if f.endswith('.onnx')
            ])
            if not candidates:
                return False
            model_path = os.path.join(self._model_dir, candidates[0])
            config_path = model_path + '.json'
            if not os.path.isfile(config_path):
                _log.warning(f'piper config not found: {config_path}')
                return False

            self._voice = piper.PiperVoice.load(
                model_path, config_path=config_path)
            _log.info(f'piper voice loaded: {os.path.basename(model_path)}')
            return True
        except Exception as e:
            _log.info(f'piper-tts skipped: {e}')
            return False

    def _try_espeak(self):
        try:
            r = subprocess.run(
                ['espeak-ng', '--version'], capture_output=True, timeout=3)
            return r.returncode == 0
        except Exception:
            return False

    @property
    def ready(self):
        return self._init_ok

    @property
    def backend(self):
        self._ensure_init()
        return self._backend

    # ── 合成 + 播放 ─────────────────────────────────────

    def speak(self, text: str, blocking: bool = False):
        """合成文本并通过 M30 扬声器播放."""
        if not self._ensure_init():
            _log.warning(f'TTS not ready, skip: {text}')
            return

        def _play():
            try:
                if self._backend == 'piper':
                    self._speak_piper(text)
                else:
                    self._speak_espeak(text)
            except Exception as e:
                _log.error(f'TTS speak error: {e}, falling back to espeak')
                try:
                    self._speak_espeak(text)
                except Exception:
                    pass

        if blocking:
            _play()
        else:
            t = threading.Thread(target=_play, daemon=True)
            t.start()

    def _speak_piper(self, text):
        """piper-tts: 合成 float32 → int16 → WAV → aplay 到 M30."""
        import numpy as np

        parts = []
        sample_rate = 22050
        for chunk in self._voice.synthesize(text):
            # audio_float_array: float32 in [-1, 1]
            arr = chunk.audio_float_array
            if arr is not None and len(arr) > 0:
                parts.append(arr)
            sample_rate = chunk.sample_rate

        if not parts:
            return

        audio = np.concatenate(parts)
        audio = np.clip(audio * 32767, -32768, 32767).astype(np.int16)

        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            wav_path = f.name

        try:
            import wave
            with wave.open(wav_path, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(sample_rate)
                wf.writeframes(audio.tobytes())

            os.system(f'aplay -q -D {self._device} {wav_path} 2>/dev/null')
        finally:
            try:
                os.unlink(wav_path)
            except OSError:
                pass

    def _speak_espeak(self, text):
        cmd = (f"espeak-ng -v zh -s 160 -a 100 --stdout "
               f"2>/dev/null | aplay -q -D {self._device} 2>/dev/null")
        proc = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE)
        try:
            proc.communicate(input=text.encode('utf-8'), timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
