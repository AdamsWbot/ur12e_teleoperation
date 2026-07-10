import io
import sys
import termios
import threading
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


class KeyboardReader(MasterReader):
    """键盘控制 — 维护虚拟关节位置，按键增量调节 — 只输出 RawDeviceData"""

    def __init__(self, cfg: KeyboardConfig):
        self._joint_step = cfg.joint_step
        self._q = [0.0] * 6
        self._keys_pressed: set[str] = set()
        self._lock = threading.Lock()
        self._running = False
        self._connected = False
        self._thread: threading.Thread | None = None
        self._old_term = None

    def connect(self) -> bool:
        if not hasattr(sys.stdin, "fileno") or not sys.stdin.isatty():
            print("[KeyboardReader] 当前终端不支持键盘输入，跳过")
            self._connected = True
            return True
        try:
            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin)
        except (termios.error, io.UnsupportedOperation):
            self._old_term = None

        self._running = True
        self._connected = True
        if hasattr(sys.stdin, "fileno") and sys.stdin.isatty():
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
        with self._lock:
            keys = list(self._keys_pressed)
            self._keys_pressed.clear()
        for ch in keys:
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
        while self._running:
            try:
                ch = sys.stdin.buffer.read(1)
                if ch:
                    decoded = ch.decode("utf-8", errors="ignore").lower()
                    if decoded in _KEY_MAP:
                        self._keys_pressed.add(decoded)
            except Exception:
                break
