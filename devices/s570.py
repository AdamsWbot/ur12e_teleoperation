import time

from src.common.types import JointState, Pose, RobotState
from src.core.master import MasterReader


class S570Reader(MasterReader):
    """通过 USB 串口从 myController S570 外骨骼读取关节数据"""

    def __init__(self, cfg: dict):
        self._usb_port = cfg.get("usb_port", "/dev/ttyUSB0")
        self._connected = False

    def connect(self) -> bool:
        # TODO: 打开 USB 串口，初始化 S570 通信
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False
        # TODO: 关闭串口

    def read(self) -> RobotState:
        timestamp = time.monotonic()
        return RobotState(
            timestamp=timestamp,
            joint=JointState(q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            tcp_pose=Pose(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0),
        )

    @property
    def is_connected(self) -> bool:
        return self._connected
