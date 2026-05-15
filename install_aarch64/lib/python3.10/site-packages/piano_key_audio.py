#!/usr/bin/env python3
"""
白键关节名 -> MIDI 音高 -> 短促合成音（电子琴风格泛音 + 衰减包络）。

播放后端优先级（Linux/虚拟机友好）：
1) **paplay / aplay** 写临时 WAV 后调用系统播放器（pulse/ALSA，一般 Ubuntu 自带，最稳）
2) sounddevice（需 PortAudio）
3) pygame.mixer

若均不可用则 backend=none。
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
import wave
from typing import List, Optional

import numpy as np

_LETTER_TO_SEMITONE = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}


def joint_name_to_midi(joint_name: str) -> Optional[int]:
    m = re.match(r"^([A-G])(\d+)_white_\d+_hinge$", joint_name)
    if not m:
        return None
    letter, octave = m.group(1), int(m.group(2))
    return 12 * (octave + 1) + _LETTER_TO_SEMITONE[letter]


def midi_to_hz(midi: int) -> float:
    return float(440.0 * (2.0 ** ((midi - 69) / 12.0)))


def _synth_tone_int16(hz: float, duration_s: float, sample_rate: int) -> np.ndarray:
    """带少量高次谐波与指数衰减，接近廉价电子琴音色。"""
    hz = float(np.clip(hz, 20.0, 12000.0))
    n = int(sample_rate * duration_s)
    t = np.arange(n, dtype=np.float64) / sample_rate
    w = 2.0 * np.pi * hz * t
    s = (
        0.52 * np.sin(w)
        + 0.26 * np.sin(2.0 * w)
        + 0.12 * np.sin(3.0 * w)
        + 0.06 * np.sin(4.0 * w)
    )
    att = max(1, int(0.002 * sample_rate))
    env = np.ones(n, dtype=np.float64)
    env[:att] = np.linspace(0.0, 1.0, att, endpoint=True)
    decay = np.exp(-4.2 * np.linspace(0.0, 1.0, n - att, endpoint=True))
    env[att:] = decay
    s = s * env * 0.88
    return (s * 32767.0).astype(np.int16)


def _synth_tone_float32(hz: float, duration_s: float, sample_rate: int) -> np.ndarray:
    y = _synth_tone_int16(hz, duration_s, sample_rate).astype(np.float64) / 32767.0
    return y.astype(np.float32)


def _alsa_play_exe() -> Optional[str]:
    """优先 PulseAudio 的 paplay，否则 ALSA 的 aplay。"""
    for name in ("paplay", "aplay"):
        p = shutil.which(name)
        if p:
            return p
    return None


def _play_pcm_wav_via_alsa(
    samples_int16: np.ndarray,
    sample_rate: int,
    *,
    blocking: bool,
) -> bool:
    """将 mono int16 PCM 写入临时 wav 并用 paplay/aplay 播放。成功返回 True。"""
    exe = _alsa_play_exe()
    if exe is None or samples_int16.size == 0:
        return False
    fd, path = tempfile.mkstemp(suffix=".wav")
    os.close(fd)
    try:
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(sample_rate)
            w.writeframes(samples_int16.astype(np.int16, copy=False).tobytes())
        base = os.path.basename(exe)
        if base == "aplay":
            cmd = [exe, "-q", path]
        else:
            cmd = [exe, path]
        if blocking:
            r = subprocess.run(
                cmd,
                timeout=30,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            if r.returncode != 0 and r.stderr:
                print(
                    f"[piano-audio] {base} 退出码={r.returncode}: {r.stderr.decode(errors='replace')[:300]}",
                    flush=True,
                )
            return r.returncode == 0
        subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        def _unlink() -> None:
            try:
                os.unlink(path)
            except OSError:
                pass

        threading.Timer(3.0, _unlink).start()
        return True
    except (OSError, subprocess.SubprocessError) as e:
        print(f"[piano-audio] 系统播放器失败: {e!r}", flush=True)
        try:
            os.unlink(path)
        except OSError:
            pass
        return False


class PianoKeyAudio:
    def __init__(
        self,
        joint_names: List[str],
        *,
        press_threshold_rad: float = -0.004,
        min_interval_s: float = 0.07,
        tone_duration_s: float = 0.18,
        sample_rate: int = 44100,
        debug: bool = False,
    ) -> None:
        self.joint_names = list(joint_names)
        self.press_threshold_rad = float(press_threshold_rad)
        self.min_interval_s = float(min_interval_s)
        self.tone_duration_s = float(tone_duration_s)
        self.sample_rate = int(sample_rate)
        self.debug = bool(debug)
        self._prev_q = np.zeros(len(self.joint_names), dtype=np.float64)
        self._last_fire = np.zeros(len(self.joint_names), dtype=np.float64)
        self._backend = self._detect_backend()
        self._pygame_inited = False

    @staticmethod
    def _sounddevice_works() -> bool:
        try:
            import sounddevice as sd

            sd.query_devices()
            return True
        except (ImportError, OSError, RuntimeError, AttributeError):
            return False

    @staticmethod
    def _pygame_available() -> bool:
        try:
            import pygame  # noqa: F401

            return True
        except ImportError:
            return False

    def _detect_backend(self) -> str:
        if _alsa_play_exe() is not None:
            return "alsa"
        if self._sounddevice_works():
            return "sounddevice"
        if self._pygame_available():
            return "pygame"
        return "none"

    @property
    def backend(self) -> str:
        return self._backend

    def _ensure_pygame(self) -> None:
        if self._pygame_inited:
            return
        import pygame

        os.environ.setdefault("SDL_AUDIODRIVER", "pulseaudio")
        try:
            pygame.mixer.pre_init(
                self.sample_rate, size=-16, channels=1, buffer=2048
            )
        except Exception:
            pass
        pygame.mixer.init(frequency=self.sample_rate, size=-16, channels=1)
        pygame.mixer.set_num_channels(16)
        self._pygame_inited = True

    def _play_hz(self, hz: float, *, blocking: bool = False) -> None:
        buf = _synth_tone_int16(hz, self.tone_duration_s, self.sample_rate)

        if self._backend == "alsa":
            _play_pcm_wav_via_alsa(buf, self.sample_rate, blocking=blocking)
            return

        if self._backend == "sounddevice":
            try:
                import sounddevice as sd

                y = _synth_tone_float32(hz, self.tone_duration_s, self.sample_rate)
                sd.play(y, samplerate=self.sample_rate, blocking=False)
                if blocking:
                    sd.wait()
                return
            except (OSError, RuntimeError):
                if self._pygame_available():
                    self._backend = "pygame"
                elif _alsa_play_exe():
                    self._backend = "alsa"
                    _play_pcm_wav_via_alsa(buf, self.sample_rate, blocking=blocking)
                else:
                    return

        if self._backend == "pygame":
            try:
                self._ensure_pygame()
                import pygame

                b = buf
                if b.ndim == 1:
                    b = np.reshape(b, (-1, 1))
                snd = pygame.sndarray.make_sound(b)
                ch = snd.play()
                if ch is not None:
                    ch.set_volume(1.0)
                if blocking:
                    while pygame.mixer.get_busy():
                        time.sleep(0.02)
            except Exception as e:
                if self.debug:
                    print(f"[piano-audio] pygame 播放失败: {e!r}", flush=True)
            return

    def play_test_beep(self, midi: int = 60) -> None:
        """启动时：用略长音 + 阻塞播放，便于确认扬声器/ALSA/Pulse。"""
        hz = midi_to_hz(int(np.clip(midi, 21, 108)))
        dur = 0.35
        buf = _synth_tone_int16(hz, dur, self.sample_rate)
        if _alsa_play_exe() is not None:
            ok = _play_pcm_wav_via_alsa(buf, self.sample_rate, blocking=True)
            if ok:
                return
        if self._backend == "sounddevice":
            try:
                import sounddevice as sd

                y = _synth_tone_float32(hz, dur, self.sample_rate)
                sd.play(y, samplerate=self.sample_rate, blocking=True)
                return
            except (OSError, RuntimeError):
                pass
        if self._pygame_available():
            self._backend = "pygame"
            self.tone_duration_s = dur
            try:
                self._play_hz(hz, blocking=True)
            finally:
                self.tone_duration_s = 0.18
            return
        print(
            "[piano-audio] 无法播放测试音：未找到 paplay/aplay，且 sounddevice/pygame 不可用",
            flush=True,
        )

    def update(
        self,
        qpos: np.ndarray,
        piano_qpos_idx: np.ndarray,
        qvel: Optional[np.ndarray] = None,
        piano_dof_idx: Optional[np.ndarray] = None,
    ) -> None:
        if self._backend == "none" or len(self.joint_names) == 0:
            return
        now = time.monotonic()
        thr = self.press_threshold_rad
        q = np.asarray(qpos[piano_qpos_idx], dtype=np.float64)
        qv = None
        if qvel is not None and piano_dof_idx is not None and len(piano_dof_idx) == len(q):
            qv = np.asarray(qvel[piano_dof_idx], dtype=np.float64)
        for i, name in enumerate(self.joint_names):
            was_pressed = self._prev_q[i] < thr
            pressed = q[i] < thr
            edge_press = pressed and not was_pressed
            fast_strike = False
            if qv is not None and not edge_press:
                fast_strike = qv[i] < -0.04 and q[i] < -0.001
            if edge_press or fast_strike:
                if now - self._last_fire[i] < self.min_interval_s:
                    continue
                self._last_fire[i] = now
                midi = joint_name_to_midi(name)
                if midi is None:
                    continue
                midi = int(np.clip(midi, 21, 108))
                hz = midi_to_hz(midi)
                if self.debug:
                    print(
                        f"[piano-audio] {name} q={q[i]:.5f} thr={thr} midi={midi} -> {hz:.1f}Hz",
                        flush=True,
                    )
                self._play_hz(hz, blocking=False)
        self._prev_q[:] = q
