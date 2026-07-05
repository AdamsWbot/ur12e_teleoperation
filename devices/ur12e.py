import time

from src.common.types import JointState, MasterReader, Pose, RobotState


class UR12eReader(MasterReader):
    """通过 RTDE 从 UR12e 主臂读取关节数据"""

    def __init__(self, cfg: dict):
        self._ip = cfg["ip"]
        self._frequency = cfg["rtde_frequency"]
        self._max_retries = cfg.get("max_retries", 3)
        self._retry_interval = cfg.get("retry_interval", 1.0)
        self._connected = False

    def connect(self) -> bool:
        for attempt in range(1, self._max_retries + 1):
            try:
                # TODO: ur_rtde.RTDE 连接
                self._connected = True
                return True
            except Exception:
                if attempt < self._max_retries:
                    time.sleep(self._retry_interval)
        return False

    def disconnect(self) -> None:
        self._connected = False
        # TODO: 断开 RTDE 连接

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
