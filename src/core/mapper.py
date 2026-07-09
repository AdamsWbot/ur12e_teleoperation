from abc import ABC, abstractmethod

from src.common.types import JointState, RobotState, RobotCommand
from src.common.config import JointLimits


class Mapper(ABC):
    """RobotState -> RobotCommand 映射器（抽象基类）

    子类只需实现 _map_joint() 定义关节空间变换；
    时间戳、delta 计算、第一帧处理由基类统一完成。

    映射公式:
        q_out[i] = direction[i] × scale × q_in[i] + offset[i]
    """

    def __init__(self, joint_limits: JointLimits):
        self._joint_limits = joint_limits
        self._scale: float = 1.0
        self._offset: tuple[float, ...] = (0.0,) * 6
        self._direction: tuple[int, ...] = (1,) * 6

    # ── 公共接口 ──────────────────────────────────

    def map(
        self,
        state: RobotState,
        prev_command: RobotCommand | None = None,
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

    def calibrate(self, joint: JointState) -> None:
        """记录当前关节位置为零点偏移（计划2 S570 校准）

        run_system.py 在启动时调用此方法，将穿戴后的初始姿态记为零位。
        校准后: offset[i] = -direction[i] × scale × joint[i]
        使得当前姿态映射后输出零位指令。
        """
        self._offset = tuple(
            -d * self._scale * q for d, q in zip(self._direction, joint)
        )

    def set_scale(self, scale: float) -> None:
        """设置全局缩放因子（默认 1.0 = 1:1 映射）"""
        self._scale = scale

    def set_offset(self, joint_idx: int, offset: float) -> None:
        """设置单关节零位偏移（弧度）。calibrate() 后如需微调可单独修改。"""
        lst = list(self._offset)
        lst[joint_idx] = offset
        self._offset = tuple(lst)

    def set_direction(self, joint_idx: int, direction: int) -> None:
        """设置单关节方向：+1（同向）或 -1（反向）

        S570 外骨骼等异构设备需要校准每个关节的旋转方向。
        也可通过 config.yaml s570.joint_direction 批量设置。
        """
        if direction not in (1, -1):
            raise ValueError(f"direction 必须为 +1 或 -1，收到 {direction}")
        lst = list(self._direction)
        lst[joint_idx] = direction
        self._direction = tuple(lst)

    def set_all_directions(self, directions: tuple[int, ...]) -> None:
        """批量设置所有 6 关节方向（来自 config.yaml s570.joint_direction）"""
        if len(directions) != 6:
            raise ValueError(f"directions 需要恰好 6 个值，实际: {len(directions)}")
        for i, d in enumerate(directions):
            if d not in (1, -1):
                raise ValueError(f"directions[{i}] 必须为 ±1，实际: {d}")
        self._direction = tuple(directions)

    # ── 子类需实现 ────────────────────────────────

    @abstractmethod
    def _map_joint(self, joint: JointState) -> JointState:
        """关节空间变换 — 子类实现具体的映射逻辑"""
        ...

    # ── 子类可用 ──────────────────────────────────

    def _apply_transform(self, joint: JointState) -> JointState:
        """应用 direction × scale + offset 的标准变换。

        JointState 支持直接迭代（__iter__），无需访问 .q。
        """
        return JointState(q=tuple(
            d * self._scale * q + o
            for d, q, o in zip(self._direction, joint, self._offset)
        ))

    # ── 基类提供 ──────────────────────────────────

    def _compute_delta(
        self,
        current: JointState,
        previous: JointState,
    ) -> JointState:
        """计算两帧目标关节的增量（JointState 可直接迭代）"""
        return JointState(q=tuple(
            c - p for c, p in zip(current, previous)
        ))


class IdentityMapper(Mapper):
    """UR12e 同构映射 — 关节空间 1:1，通过 scale/offset 可微调"""

    def _map_joint(self, joint: JointState) -> JointState:
        return self._apply_transform(joint)


class S570Mapper(Mapper):
    """S570 外骨骼 → UR12e 关节映射

    S570 是人体外骨骼（文档: docs.elephantrobotics.com/docs/myController-S570-cn/），
    每臂 7 关节（J1-J7），±180° 范围。

    ── config.yaml → 代码链接 ──────────────────────
    s570.joint_mapping   → s570.py 选6个关节 + mapper.set_joint_mapping() 存映射
    s570.joint_direction → mapper.set_all_directions()
    mapper.scale         → mapper.set_scale()
    mapper.joint_offset  → mapper.set_offset() 逐关节
    s570.enable_calibration → run_system.py 调用 mapper.calibrate()

    ── 数据流（全链路）─────────────────────────────
    s570.py: 读 7 关节 → joint_mapping 选 6 → RawDeviceData(6, 弧度)
    master.py: 时间戳 + 补零 → RobotState(6)
    mapper.py: calibrate + direction×scale+offset → RobotCommand(6)

    ── 实机校准流程 ────────────────────────────────
    1. 穿戴 S570，手臂置于 UR12e 零位姿态
    2. 启动 → auto calibrate() 记录零点偏移
    3. 逐关节活动，观察从臂跟随方向
    4. 若反向: 改 config.yaml joint_direction 对应位置为 -1
    5. 若关节错位: 改 joint_mapping 调整 S570→UR 对应关系
    """

    def __init__(self, joint_limits: JointLimits):
        super().__init__(joint_limits)
        # 与 s570.py 共享的映射配置，由 run_system.py 调用 set_joint_mapping() 同步
        self._joint_mapping: tuple[int, ...] = (1, 2, 3, 4, 5, 6)

    def set_joint_mapping(self, mapping: tuple[int, ...]) -> None:
        """设置 S570→UR 关节映射（来自 config.yaml s570.joint_mapping）。

        1-based 索引，6 个元素，值 ∈ {1..7}，无重复。
        此值需与 S570Reader._mapping 一致——两者均源自 config.yaml
        同一字段，实机调试时修改 config 即可同步。
        """
        if len(mapping) != 6:
            raise ValueError(f"joint_mapping 需要恰好 6 个元素，实际: {len(mapping)}")
        for val in mapping:
            if val not in range(1, 8):
                raise ValueError(f"joint_mapping 值必须在 1-7，实际含: {val}")
        if len(set(mapping)) != len(mapping):
            raise ValueError(f"joint_mapping 包含重复值: {mapping}")
        self._joint_mapping = tuple(mapping)

    @property
    def joint_mapping(self) -> tuple[int, ...]:
        """当前 S570→UR 关节映射（1-based，6 个元素）"""
        return self._joint_mapping

    def _map_joint(self, joint: JointState) -> JointState:
        return self._apply_transform(joint)


class KeyboardMapper(Mapper):
    """Keyboard 虚拟关节映射 — 通过 scale/offset 调整步长和零位"""

    def _map_joint(self, joint: JointState) -> JointState:
        return self._apply_transform(joint)
