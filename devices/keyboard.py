import time

from src.common.config import KeyboardConfig
from src.common.types import JointState, MasterReader, Pose, RobotState


class KeyboardReader(MasterReader):
    """键盘控制 — 维护虚拟关节位置，按键增量调节"""

    def __init__(self, cfg: KeyboardConfig):
        self._joint_step = cfg.joint_step
        self._q = [0.0] * 6
        self._connected = False

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def read(self) -> RobotState:
        # TODO: 读取键盘输入，更新 self._q
        # 按键映射: 1/q→j1±, 2/w→j2±, 3/e→j3±, 4/r→j4±, 5/t→j5±, 6/y→j6±
        timestamp = time.monotonic()
        return RobotState(
            timestamp=timestamp,
            joint=JointState(q=tuple(self._q)),
            tcp_pose=Pose(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0),
        )

    @property
    def is_connected(self) -> bool:
        return self._connected
