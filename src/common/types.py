from abc import ABC, abstractmethod
from dataclasses import dataclass


# ─── 基础数据类型 ─────────────────────────────

@dataclass(frozen=True)
class JointState:
    """6 关节状态，不可变"""
    q: tuple[float, float, float, float, float, float]

    def __post_init__(self):
        if len(self.q) != 6:
            raise ValueError(f"JointState requires exactly 6 values, got {len(self.q)}")


@dataclass(frozen=True)
class Pose:
    """TCP 位姿"""
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float


@dataclass(frozen=True)
class RobotState:
    """主臂完整状态 — master.py 输出"""
    timestamp: float
    joint: JointState
    tcp_pose: Pose


@dataclass(frozen=True)
class RobotCommand:
    """从臂控制指令 — mapper.py 输出，slave.py 输入"""
    timestamp: float
    joint: JointState
    delta: JointState   # 相对于上一帧的关节增量


@dataclass(frozen=True)
class ControlResult:
    """安全校验结果 — control.py 输出"""
    command: RobotCommand
    passed: bool
    violations: tuple[str, ...]   # 空 tuple = 全部通过


# ─── device 层输出 — 原始采集数据 ──────────────

@dataclass(frozen=True)
class RawDeviceData:
    """device 层输出 — 原始数据，不做任何标准化。device 开发者只需懂硬件协议"""
    joint: tuple[float, ...] | None = None   # 关节位置 (rad)，缺字段时 None
    tcp: Pose | None = None                  # TCP 位姿，缺字段时 None
    buttons: int = 0                         # 按钮状态位掩码，无按钮设备用默认值 0


# ─── 主端设备抽象接口 ──────────────────────────

class MasterReader(ABC):
    """所有主端设备的抽象接口 — devices/ 下所有 Reader 继承此类"""

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def read(self) -> RawDeviceData: ...

    @property
    @abstractmethod
    def is_connected(self) -> bool: ...


class MasterNormalizer(ABC):
    """数据标准化接口 — master.py 实现，将 RawDeviceData 统一为 RobotState"""

    @abstractmethod
    def normalize(self, raw: RawDeviceData) -> RobotState: ...
