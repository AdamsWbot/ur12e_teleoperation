import math
import socket
import struct
import threading

from src.common.config import S570Config
from src.common.types import MasterReader, RawDeviceData

_ANGLE_CENTER = 2048


def _decode_angle(encoded: int) -> float:
    if encoded > _ANGLE_CENTER:
        return 180.0 * (encoded - _ANGLE_CENTER) / _ANGLE_CENTER
    elif encoded < _ANGLE_CENTER:
        return -180.0 * (_ANGLE_CENTER - encoded) / _ANGLE_CENTER
    return 0.0


class S570Reader(MasterReader):
    """通过 TCP 桥接从 S570 读取 6 关节数据 — 只输出 RawDeviceData"""

    def __init__(self, cfg: S570Config):
        self._usb_port = cfg.usb_port
        self._arm = 1 if cfg.active_arm == "left" else 2
        self._mapping: tuple[int, ...] = cfg.joint_mapping
        if len(self._mapping) != 6:
            raise ValueError(
                f"joint_mapping 需要恰好 6 个元素，实际: {len(self._mapping)}"
            )
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._connected = False

    def connect(self) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(3)
            self._sock.connect(("127.0.0.1", 15570))
            self._sock.settimeout(1)
            self._connected = True
            return True
        except (socket.error, OSError):
            self._connected = False
            return False

    def disconnect(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._connected = False

    def read(self) -> RawDeviceData:
        """排空 TCP 缓冲后读取最新的 6 关节数据。"""
        raw = self._drain_and_get_latest()
        if not raw:
            raise IOError("S570: no data from bridge")
        angles_deg = self._parse_angles(raw)
        joint = tuple(
            math.radians(angles_deg[idx - 1]) for idx in self._mapping
        )
        return RawDeviceData(joint=joint, tcp=None)

    def _drain_and_get_latest(self) -> bytes | None:
        """排空缓冲区，返回最后一帧完整数据。"""
        self._sock.settimeout(0)
        latest = None
        drained_any = False
        while True:
            try:
                frame = self._recv_frame()
                if frame:
                    latest = frame
                    drained_any = True
            except (socket.timeout, BlockingIOError):
                break
        if not drained_any:
            # 没有积压，等一帧
            self._sock.settimeout(2)
            latest = self._recv_frame()
        self._sock.settimeout(1)
        return latest

    @property
    def is_connected(self) -> bool:
        return self._connected and self._sock is not None

    def sync_initial_position(self, joint: tuple[float, ...]) -> None:
        pass

    def _recv_frame(self) -> bytes:
        # 读取长度头 (2 bytes big-endian)
        header = b""
        while len(header) < 2:
            chunk = self._sock.recv(2 - len(header))
            if not chunk:
                raise IOError("TCP bridge closed")
            header += chunk
        length = struct.unpack("!H", header)[0]
        # 读取 payload
        payload = b""
        while len(payload) < length:
            chunk = self._sock.recv(length - len(payload))
            if not chunk:
                raise IOError("TCP bridge closed")
            payload += chunk
        return payload

    def _parse_angles(self, data: bytes) -> list[float]:
        hex_str = data.hex()[2:]  # skip command echo byte
        # 解析足够覆盖 joint_mapping 最大索引的角度值
        n = max(self._mapping)
        angles = []
        for i in range(n):
            hex_val = hex_str[i * 4 : (i + 1) * 4]
            if len(hex_val) < 4:
                break
            encoded = int(hex_val, 16)
            angles.append(_decode_angle(encoded))
        return angles
