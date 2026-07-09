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
    每臂 7 关节（J1-J7），±180° 范围。映射到 UR12e 的 6 关节时，
    需丢弃 1 个关节（默认丢弃 J7 = drop_joint=6）。

    关节数据流:
        s570.py: 完整 7 关节 → RawDeviceData
        master.py: 根据 drop_index 丢弃 1 个 → RobotState(6 关节)
        mapper.py: 对 6 关节做 direction×scale+offset 变换 → RobotCommand

    典型校准流程:
        1. 穿戴 S570，将手臂置于 UR12e 的零位姿态
        2. 逐一活动每个关节，观察从臂跟随方向
        3. 若关节反向运动，调用 set_direction(idx, -1) 反转
        4. 若不明确哪个是 J7，调用 set_drop_joint(idx) 尝试不同丢弃位置
    """

    def __init__(self, joint_limits: JointLimits, drop_joint: int = 6):
        super().__init__(joint_limits)
        if not (0 <= drop_joint <= 6):
            raise ValueError(f"drop_joint 必须在 0-6，收到 {drop_joint}")
        self._drop_joint = drop_joint

    @property
    def drop_joint(self) -> int:
        """要丢弃的 S570 关节索引（0-based，默认 6 = J7）"""
        return self._drop_joint

    def set_drop_joint(self, idx: int) -> None:
        """设置要丢弃的关节索引（0-based，0=J1, 6=J7）。

        S570 有 7 个关节，UR12e 只有 6 个。通过此方法选择丢弃哪个。
        默认丢弃 J7（最末关节），不确定映射关系时可尝试其他值。
        """
        if not (0 <= idx <= 6):
            raise ValueError(f"drop_joint 必须在 0-6，收到 {idx}")
        self._drop_joint = idx

    def _map_joint(self, joint: JointState) -> JointState:
        return self._apply_transform(joint)


class KeyboardMapper(Mapper):
    """Keyboard 虚拟关节映射 — 通过 scale/offset 调整步长和零位"""

    def _map_joint(self, joint: JointState) -> JointState:
        return self._apply_transform(joint)
