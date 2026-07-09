from abc import ABC, abstractmethod

from src.common.types import JointState, RobotState, RobotCommand
from src.common.config import JointLimits


class Mapper(ABC):
    """RobotState -> RobotCommand 映射器（抽象基类）

    子类只需实现 _map_joint() 定义关节空间变换；
    时间戳、delta 计算、第一帧处理由基类统一完成。

    映射公式（子类可覆写 _apply_transform）:
        q_out[i] = _direction[i] * scale * q_in[i] + offset[i]
    """

    def __init__(self, joint_limits: JointLimits):
        self._joint_limits = joint_limits
        self._scale: float = 1.0
        self._offset: tuple[float, ...] = (0.0,) * 6
        # per-joint direction: +1 (同向) or -1 (反向), 子类可覆写
        self._direction: tuple[int, ...] = (1,) * 6

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
        """设置全局缩放因子（默认 1.0 = 1:1 映射）"""
        self._scale = scale

    def set_offset(self, joint_idx: int, offset: float) -> None:
        """设置单关节零位偏移（弧度）"""
        lst = list(self._offset)
        lst[joint_idx] = offset
        self._offset = tuple(lst)

    def set_direction(self, joint_idx: int, direction: int) -> None:
        """设置单关节方向：+1（同向）或 -1（反向）

        S570 外骨骼等异构设备需要校准每个关节的旋转方向。
        """
        if direction not in (1, -1):
            raise ValueError(f"direction 必须为 +1 或 -1，收到 {direction}")
        lst = list(self._direction)
        lst[joint_idx] = direction
        self._direction = tuple(lst)

    # ── 子类需实现 ────────────────────────────────

    @abstractmethod
    def _map_joint(self, joint: JointState) -> JointState:
        """关节空间变换 — 子类实现具体的映射逻辑"""
        ...

    # ── 子类可用 ──────────────────────────────────

    def _apply_transform(self, joint: JointState) -> JointState:
        """应用 direction × scale + offset 的标准变换。

        大多数子类的 _map_joint() 只需调用此方法即可。
        """
        return JointState(q=tuple(
            d * self._scale * q + o
            for d, q, o in zip(self._direction, joint.q, self._offset)
        ))

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
    """UR12e 同构映射 — 关节空间 1:1，通过 scale/offset 可微调"""

    def _map_joint(self, joint: JointState) -> JointState:
        return self._apply_transform(joint)


class S570Mapper(Mapper):
    """S570 外骨骼 → UR12e 关节映射

    S570 是人体外骨骼（文档: docs.elephantrobotics.com/docs/myController-S570-cn/），
    6+6 DOF，每关节 ±180°。人体手臂关节旋转方向与 UR12e 机器人坐标系可能不一致，
    需要通过 set_direction() 逐关节校准符号。

    典型校准流程:
        1. 穿戴 S570，将手臂置于 UR12e 的零位姿态
        2. 逐一活动每个关节，观察从臂跟随方向
        3. 若关节反向运动，调用 set_direction(idx, -1) 反转
    """

    def _map_joint(self, joint: JointState) -> JointState:
        return self._apply_transform(joint)


class KeyboardMapper(Mapper):
    """Keyboard 虚拟关节映射 — 通过 scale/offset 调整步长和零位"""

    def _map_joint(self, joint: JointState) -> JointState:
        return self._apply_transform(joint)
