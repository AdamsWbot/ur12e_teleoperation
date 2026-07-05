import threading

from pynput.keyboard import Key, KeyCode, Listener

from src.common.config import KeyboardConfig
from src.common.types import MasterReader, RawDeviceData

# ─── 按键 → 关节索引映射 ────────────────────────
# 数字键=正向(+step)，字母键=负向(−step)

_KEY_MAP: dict[str, tuple[int, int]] = {
    # (joint_index, direction): direction=+1 正向, -1 负向
    "1": (0, +1), "q": (0, -1),
    "2": (1, +1), "w": (1, -1),
    "3": (2, +1), "e": (2, -1),
    "4": (3, +1), "r": (3, -1),
    "5": (4, +1), "t": (4, -1),
    "6": (5, +1), "y": (5, -1),
}


def _key_to_char(key) -> str | None:
    """将 pynput Key 转为规范化的字符键字符串。"""
    if isinstance(key, KeyCode) and key.char is not None:
        return key.char
    return None


class KeyboardReader(MasterReader):
    """键盘控制 — 维护虚拟关节位置，按键增量调节 — 只输出 RawDeviceData"""

    def __init__(self, cfg: KeyboardConfig):
        self._joint_step = cfg.joint_step
        self._q = [0.0] * 6
        self._keys_pressed: set[str] = set()
        self._lock = threading.Lock()
        self._listener: Listener | None = None
        self._connected = False

    # ─── MasterReader 接口 ────────────────────────

    def connect(self) -> bool:
        try:
            self._listener = Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            self._listener.start()
            self._connected = True
            return True
        except Exception:
            self._connected = False
            return False

    def disconnect(self) -> None:
        if self._listener is not None:
            self._listener.stop()
            self._listener = None
        self._connected = False

    def read(self) -> RawDeviceData:
        self._update_joints()
        with self._lock:
            joint = tuple(self._q)
        return RawDeviceData(joint=joint, tcp=None)

    @property
    def is_connected(self) -> bool:
        return self._connected and self._listener is not None and self._listener.is_alive()

    # ─── 内部 ─────────────────────────────────────

    def _on_press(self, key):
        ch = _key_to_char(key)
        if ch is not None:
            self._keys_pressed.add(ch)

    def _on_release(self, key):
        ch = _key_to_char(key)
        if ch is not None:
            self._keys_pressed.discard(ch)

    def _update_joints(self):
        if not self._keys_pressed:
            return
        with self._lock:
            for ch in self._keys_pressed:
                if ch in _KEY_MAP:
                    idx, direction = _KEY_MAP[ch]
                    self._q[idx] += direction * self._joint_step
