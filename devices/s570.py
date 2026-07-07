import math
import threading
import time

import serial

from src.common.config import S570Config
from src.common.types import MasterReader, RawDeviceData

# ─── S570 串口协议常量（来源: pymycobot exoskeleton.py, MIT License）───

_BAUDRATE = 1_000_000
_FRAME_HEAD = b"\xfe\xfe"
_FRAME_TAIL = 0xFA

# 命令
_CMD_GET_ALL_DATA = bytes([0xFE, 0xFE, 0x02, 0x01, 0xFA])
_CMD_GET_ARM_DATA = bytes([0xFE, 0xFE, 0x03, 0x02])  # + [arm] + [0xFA]

# 角度编码基准
_ANGLE_CENTER = 2048


def _hex_to_signed_decimal(hex_str: str, bits: int = 16) -> int:
    """将 hex 字符串转为有符号整数。"""
    value = int(hex_str, 16)
    if value >= 2 ** (bits - 1):
        value -= 2**bits
    return value


def _decode_angle(encoded: int) -> float:
    """2048-中心编码 → 角度（度）。"""
    if encoded > _ANGLE_CENTER:
        return 180.0 * (encoded - _ANGLE_CENTER) / _ANGLE_CENTER
    elif encoded < _ANGLE_CENTER:
        return -180.0 * (_ANGLE_CENTER - encoded) / _ANGLE_CENTER
    return 0.0


class S570Reader(MasterReader):
    """通过 USB 串口从 myController S570 外骨骼读取关节数据 — 只输出 RawDeviceData"""

    def __init__(self, cfg: S570Config):
        self._usb_port = cfg.usb_port
        self._ser: serial.Serial | None = None
        self._lock = threading.Lock()
        self._connected = False

    # ─── MasterReader 接口 ────────────────────────

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(
                port=self._usb_port,
                baudrate=_BAUDRATE,
                timeout=0.1,
            )
            self._connected = True
            return True
        except (serial.SerialException, OSError):
            self._connected = False
            return False

    def disconnect(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None
        self._connected = False

    def read(self) -> RawDeviceData:
        """读取左臂关节数据和按钮状态，返回 RawDeviceData。

        S570 协议（命令 0x02）返回 7 个关节 + 按钮/摇杆状态 + 笛卡尔坐标。
        关节取前 6 个（J7 丢弃），按钮以位掩码形式保留。
        """
        data = self._send_command(bytes([0x02, 0x01]))  # GET_ARM_DATA arm=1
        angles, buttons = self._parse_response(data)
        joint = tuple(math.radians(a) for a in angles[:6])
        return RawDeviceData(joint=joint, tcp=None, buttons=buttons)

    @property
    def is_connected(self) -> bool:
        return self._connected and self._ser is not None and self._ser.is_open

    # ─── 协议层（内嵌）────────────────────────────

    def _send_command(self, payload: bytes) -> str:
        """发送命令帧，返回响应 hex 字符串。"""
        frame = _FRAME_HEAD + bytes([len(payload) + 1]) + payload + bytes([_FRAME_TAIL])
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            time.sleep(0.001)
            # 读取响应帧头
            head = self._ser.read(2)
            if head != _FRAME_HEAD:
                raise IOError(f"S570: bad frame head: {head.hex()}")
            # 读取数据长度字节
            data_len_byte = self._ser.read(1)
            if not data_len_byte:
                raise IOError("S570: timeout reading data length")
            # 读取剩余数据 + 尾字节
            remaining = self._ser.read(data_len_byte[0])
            if not remaining or remaining[-1] != _FRAME_TAIL:
                raise IOError("S570: incomplete frame")
            return remaining[:-1].hex()

    def _parse_response(self, data: str) -> tuple[list[float], int]:
        """从 hex 响应中解析关节角度（度）和按钮状态。

        响应格式（命令 0x02，来源 S570 串口协议文档）:
            回显(1B) | J1..J7(各2B) | 按钮(1B) | 摇杆X(1B) | 摇杆Y(1B) | 笛卡尔(12B)

        按钮位掩码: bit1=摇杆按钮, bit2=按钮1, bit3=按钮2
        """
        # 跳过前 2 hex 字符（命令回显字节）
        payload = data[2:]
        angles = []
        for i in range(7):
            hex_val = payload[i * 4 : (i + 1) * 4]
            if len(hex_val) < 4:
                break
            encoded = _hex_to_signed_decimal(hex_val)
            angles.append(_decode_angle(encoded))

        # 按钮状态位于 7 关节之后（偏移 28 hex 字符，1 字节）
        btn_offset = 7 * 4  # 28
        buttons = 0
        if len(payload) >= btn_offset + 2:
            buttons = int(payload[btn_offset : btn_offset + 2], 16)

        return angles, buttons
