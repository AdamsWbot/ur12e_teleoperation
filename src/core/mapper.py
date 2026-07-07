from abc import ABC, abstractmethod

from src.common.types import JointState, RobotState, RobotCommand
from src.common.config import JointLimits


class Mapper(ABC):
    """RobotState -> RobotCommand 映射器（抽象基类）

    子类只需实现 _map_joint() 定义关节空间变换；
    时间戳、delta 计算、第一帧处理由基类统一完成。
    """

    def __init__(self, joint_limits: JointLimits):
        self._joint_limits = joint_limits
        self._scale: float = 1.0
        self._offset: tuple[float, ...] = (0.0,) * 6

    # ── 公共接口 ──────────────────────────────────

    def map(
        self,
        state: RobotState,
        prev_command: RobotCommand | None,
    ) -> RobotCommand:
        """将 RobotState 映射为 RobotCommand"""
        target_joint = self._map_joint(state.joint)

        if prev_command is None:
            previous_joint = target_joint
        else:
            previous_joint = prev_command.joint

        delta = self._compute_delta(target_joint, previous_joint)

        return RobotCommand(
            timestamp=state.timestamp,
            joint=target_joint,
            delta=delta,
        )

    def set_scale(self, scale: float) -> None:
        """预留接口：设置映射比例"""
        self._scale = scale

    def set_offset(self, joint_idx: int, offset: float) -> None:
        """预留接口：设置关节偏移"""
        lst = list(self._offset)
        lst[joint_idx] = offset
        self._offset = tuple(lst)

    # ── 子类需实现 ────────────────────────────────

    @abstractmethod
    def _map_joint(self, joint: JointState) -> JointState:
        """关节空间变换 — 子类实现具体的映射逻辑"""
        ...

    # ── 基类提供 ──────────────────────────────────

    def _compute_delta(
        self,
        current: JointState,
        previous: JointState,
    ) -> JointState:
        """计算两帧目标关节的增量"""
        return JointState(q=tuple(
            c - p for c, p in zip(current.q, previous.q)
        ))


class IdentityMapper(Mapper):
    """UR12e 同构映射 — 关节空间 1:1"""

    def _map_joint(self, joint: JointState) -> JointState:
        return joint


class S570Mapper(Mapper):
    """S570 外骨骼映射 — 关节重映射（P3 实现具体逻辑）"""

    def _map_joint(self, joint: JointState) -> JointState:
        return joint  # TODO: 关节索引重映射（如 j1→−j1 等）


class KeyboardMapper(Mapper):
    """Keyboard 虚拟关节映射 — 直接透传"""

    def _map_joint(self, joint: JointState) -> JointState:
        return joint