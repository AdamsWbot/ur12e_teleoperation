"""键盘控制 — cbreak 终端读取，时间戳追踪按键状态。

修复: _keys_pressed 不再每帧 clear()，改用最后出现时间追踪——
      50ms 内无新字节才算释放，消除 autorepeat 和 pipeline 频率
      不匹配造成的 ON-OFF 脉冲振荡。
"""

import io
import os
import signal
import sys
import termios
import threading
import time
import tty

from src.common.config import KeyboardConfig
from src.common.types import MasterReader, RawDeviceData

_KEY_MAP: dict[str, tuple[int, int]] = {
    "1": (0, +1), "q": (0, -1),
    "2": (1, +1), "w": (1, -1),
    "3": (2, +1), "e": (2, -1),
    "4": (3, +1), "r": (3, -1),
    "5": (4, +1), "t": (4, -1),
    "6": (5, +1), "y": (5, -1),
}

# 按键释放判定: 超过此时间未收到新字节即认为已释放
_HOLD_TIMEOUT = 0.3  # 300ms > 终端自动重复初始延迟 (250-500ms)


class KeyboardReader(MasterReader):
    """键盘控制 — 时间戳追踪，按键期间连续输出，无脉冲"""

    def __init__(self, cfg: KeyboardConfig):
        self._joint_step = cfg.joint_step
        self._q = [0.0] * 6
        self._keys_ts: dict[str, float] = {}    # ch → 最后出现时间
        self._lock = threading.Lock()
        self._running = False
        self._connected = False
        self._thread: threading.Thread | None = None
        self._old_term = None

    def connect(self) -> bool:
        if not hasattr(sys.stdin, "fileno") or not sys.stdin.isatty():
            print("[KeyboardReader] 终端不支持键盘输入")
            self._connected = False
            return False
        try:
            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)
        except (termios.error, io.UnsupportedOperation):
            self._old_term = None

        self._running = True
        self._connected = True
        self._thread = threading.Thread(target=self._listen_keys, daemon=True)
        self._thread.start()
        return True

    def disconnect(self) -> None:
        self._running = False
        if self._old_term is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass
        self._connected = False

    def read(self) -> RawDeviceData:
        now = time.monotonic()
        with self._lock:
            # 淘汰 >50ms 未出现的按键
            stale = [ch for ch, ts in self._keys_ts.items() if now - ts > _HOLD_TIMEOUT]
            for ch in stale:
                del self._keys_ts[ch]
            active = list(self._keys_ts.keys())

        for ch in active:
            if ch in _KEY_MAP:
                idx, direction = _KEY_MAP[ch]
                self._q[idx] += direction * self._joint_step

        with self._lock:
            joint = tuple(self._q)
        return RawDeviceData(joint=joint, tcp=None)

    @property
    def is_connected(self) -> bool:
        return self._connected

    def sync_initial_position(self, joint: tuple[float, ...]) -> None:
        with self._lock:
            self._q = list(joint[:6])

    def _listen_keys(self) -> None:
        fd = sys.stdin.fileno()
        while self._running:
            try:
                b = os.read(fd, 1)
                if not b:
                    break
                if b[0] == 0x03:         # Ctrl+C
                    os.kill(os.getpid(), signal.SIGINT)
                    return
                decoded = b.decode("utf-8", errors="ignore").lower()
                if decoded in _KEY_MAP:
                    with self._lock:
                        self._keys_ts[decoded] = time.monotonic()
            except Exception:
                break
