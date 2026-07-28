#!/usr/bin/env python3
"""TTS 引擎 — WAV 缓存 + 多后端离线中文语音合成, 输出到 M30 USB 扬声器.

后端优先级:
  1. piper-tts   — 神经网络合成 → 首次合成后 WAV 缓存, 后续 aplay 直放
  2. espeak-ng   — 规则合成, 零模型下载, 永远可用 (兜底)

单例模式, speak() 非阻塞 (后台线程播放). 首次合成慢 (~10-15s),
后续同文本秒级响应 (~0.1s).

配置:
    PIPER_MODEL_DIR — piper 模型目录, 默认 /home/sunrise/tts_models/piper-zh
    TTS_CACHE_DIR   — WAV 缓存目录, 默认 /home/sunrise/tts_models/piper-cache
    TTS_DEVICE      — ALSA 输出设备, 默认 plughw:0,0 (M30 USB)
"""

import os
import logging
import threading
import subprocess
import hashlib

import numpy as np

_log = logging.getLogger('tts_engine')


class TTSEngine:
    """单例 TTS 引擎, 惰性初始化 + WAV 缓存."""

    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        self._voice = None
        self._model_dir = os.environ.get(
            'PIPER_MODEL_DIR',
            '/home/sunrise/tts_models/piper-zh')
        self._cache_dir = os.environ.get(
            'TTS_CACHE_DIR',
            '/home/sunrise/tts_models/piper-cache')
        self._device = os.environ.get('TTS_DEVICE', 'plughw:0,0')
        self._backend = 'none'
        self._init_ok = False
        self._synthesizing = set()  # 正在合成的文本 (防重复合成)

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
                os.makedirs(self._cache_dir, exist_ok=True)
                _log.info(f'TTS ready: piper-tts, cache={self._cache_dir}')
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
            # 限制 ONNX 线程数, 避免与 BPU 推理争抢 CPU
            import onnxruntime
            onnxruntime.set_default_logger_severity(3)  # suppress warnings
            os.environ.setdefault('OMP_NUM_THREADS', '2')

            import piper
            candidates = sorted([
                f for f in os.listdir(self._model_dir)
                if f.endswith('.onnx')
            ])
            if not candidates:
                return False
            model_path = os.path.join(self._model_dir, candidates[0])
            config_path = model_path + '.json'
            if not os.path.isfile(config_path):
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

    # ── WAV 缓存 ────────────────────────────────────────

    def _cache_path(self, text):
        h = hashlib.md5(text.encode('utf-8')).hexdigest()[:12]
        return os.path.join(self._cache_dir, f'{h}.wav')

    def _cached(self, text):
        return os.path.isfile(self._cache_path(text))

    def _synthesize_and_cache(self, text):
        """合成文本并保存 WAV 到缓存. 耗时操作, 在后台线程调用."""
        _log.info(f'Synthesizing: "{text}"')
        parts = []
        sample_rate = 22050
        for chunk in self._voice.synthesize(text):
            arr = chunk.audio_float_array
            if arr is not None and len(arr) > 0:
                parts.append(arr)
            sample_rate = chunk.sample_rate

        if not parts:
            _log.error(f'Synthesis produced no audio: "{text}"')
            return False

        audio = np.concatenate(parts)
        audio = np.clip(audio * 32767, -32768, 32767).astype(np.int16)

        import wave
        path = self._cache_path(text)
        # 写临时文件再 rename (原子操作)
        tmp = path + '.tmp'
        with wave.open(tmp, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            wf.writeframes(audio.tobytes())
        os.rename(tmp, path)
        _log.info(f'Cached: "{text}" → {os.path.basename(path)}')
        return True

    # ── 合成 + 播放 ─────────────────────────────────────

    def speak(self, text: str, blocking: bool = False):
        """播放文本 (缓存命中 → 直接 aplay, 未命中 → 后台合成+播放)."""
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
                _log.error(f'TTS speak error: {e}')
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
        cache_path = self._cache_path(text)

        if os.path.isfile(cache_path):
            # 缓存命中 → 直接播放
            os.system(f'aplay -q -D {self._device} {cache_path} 2>/dev/null')
            return

        # 未命中 → 需要合成 + 缓存
        # 防止多线程同时合成同一文本
        with self._lock:
            if text in self._synthesizing:
                return  # 已在合成中, 跳过一次
            self._synthesizing.add(text)

        try:
            ok = self._synthesize_and_cache(text)
            if ok:
                os.system(
                    f'aplay -q -D {self._device} {cache_path} 2>/dev/null')
            else:
                self._speak_espeak(text)
        finally:
            with self._lock:
                self._synthesizing.discard(text)

    def _speak_espeak(self, text):
        cmd = (f"espeak-ng -v zh -s 160 -a 100 --stdout "
               f"2>/dev/null | aplay -q -D {self._device} 2>/dev/null")
        proc = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE)
        try:
            proc.communicate(input=text.encode('utf-8'), timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    # ── 预热 (启动时后台预合成全部已知短语) ──────────────

    def warmup(self, phrases: list[str]):
        """后台线程预合成指定短语列表."""
        if not self._ensure_init() or self._backend != 'piper':
            return

        def _warm():
            for text in phrases:
                if not self._cached(text):
                    try:
                        self._synthesize_and_cache(text)
                    except Exception as e:
                        _log.error(f'Warmup failed "{text}": {e}')

        t = threading.Thread(target=_warm, daemon=True)
        t.start()
