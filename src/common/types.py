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


# ─── 主端设备抽象接口 ──────────────────────────

class MasterReader(ABC):
    """所有主端设备的抽象接口 — devices/ 下所有 Reader 继承此类"""

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def read(self) -> RobotState: ...

    @property
    @abstractmethod
    def is_connected(self) -> bool: ...
