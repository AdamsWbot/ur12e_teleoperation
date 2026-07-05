import time

from rtde_receive import RTDEReceiveInterface

from src.common.config import MasterConfig
from src.common.types import MasterReader, Pose, RawDeviceData


class UR12eReader(MasterReader):
    """通过 RTDE 从 UR12e 主臂读取关节数据 — 只输出 RawDeviceData，不关心 RobotState"""

    def __init__(self, cfg: MasterConfig):
        self._ip = cfg.ip
        self._frequency = cfg.rtde_frequency
        self._max_retries = cfg.max_retries
        self._retry_interval = cfg.retry_interval
        self._connected = False
        self._rtde = None

    def connect(self) -> bool:
        for attempt in range(1, self._max_retries + 1):
            try:
                self._rtde = RTDEReceiveInterface(self._ip, self._frequency)
                self._connected = self._rtde.isConnected()
                return self._connected
            except Exception:
                if attempt < self._max_retries:
                    time.sleep(self._retry_interval)
        return False

    def disconnect(self) -> None:
        if self._rtde is not None:
            self._rtde.disconnect()
            self._rtde = None
        self._connected = False

    def read(self) -> RawDeviceData:
        actual_q = self._rtde.getActualQ()
        tcp = self._rtde.getActualTCPPose()
        return RawDeviceData(
            joint=tuple(actual_q),
            tcp=Pose(
                x=tcp[0], y=tcp[1], z=tcp[2],
                rx=tcp[3], ry=tcp[4], rz=tcp[5],
            ),
        )

    @property
    def is_connected(self) -> bool:
        return self._rtde is not None and self._rtde.isConnected()
