from src.common.types import JointState, RobotState, RobotCommand
from src.common.config import JointLimits  

class Mapper:
    """RobotState -> RobotCommand 映射器"""

    def __init__(self, joint_limits: JointLimits):
        """初始化 Mapper"""
        self._joint_limits = joint_limits
        self._scale: float = 1.0
        self._offset: tuple[float, ...] = (0.0,) * 6

    def map(
        self,
        state: RobotState,
        prev_command: RobotCommand | None,
    ) -> RobotCommand:
        """将 RobotState 映射为 RobotCommand"""

        # 当前阶段：同构映射，直接调用内部映射逻辑
        target_joint = self._map_joint(state.joint)

        # 第一帧增量为 0
        if prev_command is None:
            previous_joint = target_joint
        else:
            previous_joint = prev_command.joint

        delta = self._compute_delta(
            target_joint,
            previous_joint,
        )

        return RobotCommand(
            timestamp=state.timestamp,
            joint=target_joint,
            delta=delta,
        )

    def _map_joint(self, joint: JointState) -> JointState:
        """关节映射：同构机械臂 1:1 映射"""
        return joint

    def _compute_delta(
        self,
        current: JointState,
        previous: JointState,
    ) -> JointState:
        """计算两帧目标关节的增量"""
        delta = tuple(
            current_value - previous_value
            for current_value, previous_value
            in zip(current.q, previous.q)
        )
        # 显式使用关键字参数 q=delta 确保符合 dataclass 定义
        return JointState(q=delta)

    def set_scale(self, scale: float) -> None:
        """预留接口：设置映射比例"""
        self._scale = scale

    def set_offset(self, joint_idx: int, offset: float) -> None:
        """预留接口：设置关节偏移"""
        pass