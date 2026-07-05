import time

from src.common.config import S570Config
from src.common.types import MasterReader, RawDeviceData


class S570Reader(MasterReader):
    """通过 USB 串口从 myController S570 外骨骼读取关节数据 — 只输出 RawDeviceData"""

    def __init__(self, cfg: S570Config):
        self._usb_port = cfg.usb_port
        self._connected = False

    def connect(self) -> bool:
        # TODO: 打开 USB 串口，初始化 S570 通信
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False
        # TODO: 关闭串口

    def read(self) -> RawDeviceData:
        # TODO: 读取 S570 编码器数据 (6 关节角度)，无 TCP 信息
        return RawDeviceData(
            joint=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            tcp=None,
        )

    @property
    def is_connected(self) -> bool:
        return self._connected
