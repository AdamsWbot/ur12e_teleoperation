import math
from src.common.types import RobotCommand, ControlResult, JointState
from src.common.config import ControlConfig

class SafetyController:
    """安全校验层 — 在校验通过前阻止指令下发到从臂"""

    def __init__(self, cfg: ControlConfig):
        self._cfg = cfg
        self._dt = 1.0 / cfg.frequency if cfg.frequency > 0 else 0.01

    def validate(self, command: RobotCommand) -> ControlResult:
        """校验 RobotCommand 并返回 ControlResult"""
        violations = []
        q_list = list(command.joint.q)
        passed = True

        # 1. 异常值检测 (NaN / Inf)
        for val in q_list:
            if not math.isfinite(val):
                violations.append("Invalid value (NaN/Inf) detected")
                passed = False
                return ControlResult(command=self.emergency_stop(), passed=False, violations=tuple(violations))

        # 2. 关节限位检查
        if self._cfg.enable_joint_limit:
            limits = [
                self._cfg.joint_limits.j1, self._cfg.joint_limits.j2,
                self._cfg.joint_limits.j3, self._cfg.joint_limits.j4,
                self._cfg.joint_limits.j5, self._cfg.joint_limits.j6
            ]
            for i in range(6):
                min_l, max_l = limits[i]
                if q_list[i] < min_l:
                    q_list[i] = min_l
                    violations.append(f"Joint {i+1} exceed min limit")
                    passed = False
                elif q_list[i] > max_l:
                    q_list[i] = max_l
                    violations.append(f"Joint {i+1} exceed max limit")
                    passed = False

        # 3. 速度限幅检查
        if self._cfg.enable_velocity_limit and self._dt > 0:
            for i in range(6):
                vel = abs(command.delta.q[i]) / self._dt
                if vel > self._cfg.max_joint_velocity:
                    violations.append(f"Joint {i+1} exceeds max velocity")
                    passed = False
                    # 简单截断处理：若超速，此处逻辑后续需要调整
        
        # 构造安全指令（若违规，使用截断后的 q）
        safe_command = RobotCommand(
            timestamp=command.timestamp,
            joint=JointState(q=tuple(q_list)),
            delta=command.delta
        )

        return ControlResult(
            command=safe_command,
            passed=passed,
            violations=tuple(violations)
        )

    def emergency_stop(self) -> RobotCommand:
        """生成紧急停止指令（全零关节位置与速度）"""
        return RobotCommand(
            timestamp=0.0,
            joint=JointState(q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            delta=JointState(q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        )