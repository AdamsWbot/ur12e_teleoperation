from abc import ABC, abstractmethod
from dataclasses import dataclass


# ─── 基础数据类型 ─────────────────────────────

@dataclass(frozen=True)
class JointState:
    """6 关节状态，不可变。

    支持便利接口：可直接迭代、索引和取长度，无需访问 .q 属性。
      - list(state) → 6 元素列表
      - state[3]   → 第 4 关节值
      - len(state) → 总是 6
      - for x in state: ...
    """
    q: tuple[float, float, float, float, float, float]

    def __post_init__(self):
        if len(self.q) != 6:
            raise ValueError(
                f"JointState 需要恰好 6 个关节值，实际: {len(self.q)}"
            )
        for i, v in enumerate(self.q):
            if not isinstance(v, (int, float)):
                raise TypeError(
                    f"JointState 第 {i + 1} 个关节值类型错误: "
                    f"期望 int 或 float，实际 {type(v).__name__} (值={v!r})"
                )

    # ── 便利接口 — 委托给 self.q ──────────────────

    def __iter__(self):
        return iter(self.q)

    def __getitem__(self, index):
        return self.q[index]

    def __len__(self) -> int:
        return 6


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
    joint: tuple[float, ...] | None = None            # 关节位置 (rad)，缺字段时 None
    joint_secondary: tuple[float, ...] | None = None  # [计划2] 副侧关节（S570 另一臂）；不支持的设备为 None
    tcp: Pose | None = None                           # TCP 位姿，缺字段时 None
    buttons: int = 0                                  # 按钮状态位掩码，无按钮设备用默认值 0
    axes: tuple[float, ...] | None = None             # [计划2] 模拟轴（摇杆），不支持的设备为 None
    imu: tuple[float, ...] | None = None              # [计划2] IMU 数据，不支持的设备为 None


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
