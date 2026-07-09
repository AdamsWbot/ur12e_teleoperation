import math
from enum import Enum, auto

from src.common.types import RobotCommand, ControlResult, JointState
from src.common.config import ControlConfig


class EStopState(Enum):
    """急停状态机状态"""
    NORMAL = auto()       # 正常运行
    ESTOP = auto()        # 急停已触发：NaN/Inf 或外部 emergency_stop()
    RECOVERED = auto()    # 已恢复但未手动复位：数据恢复干净，需 reset() 回到 NORMAL


class SafetyController:
    """安全校验层 — 在校验通过前阻止指令下发到从臂

    急停状态机：
        NORMAL → ESTOP:  validate() 检测到 NaN/Inf，或外部调用 emergency_stop()
        ESTOP → RECOVERED: 下一帧数据恢复有效（无 NaN/Inf），但指令仍被拦截
        RECOVERED → NORMAL: 调用 reset() 手动确认安全
    """

    def __init__(self, cfg: ControlConfig):
        self._cfg = cfg
        self._dt = 1.0 / cfg.frequency if cfg.frequency > 0 else 0.01
        self._estop_state = EStopState.NORMAL

    # ─── 急停状态机 ─────────────────────────────────

    @property
    def estop_status(self) -> EStopState:
        """当前急停状态"""
        return self._estop_state

    def emergency_stop(self, timestamp: float = 0.0) -> RobotCommand:
        """生成紧急停止指令并将状态机置为 ESTOP

        全零关节位置与速度，从臂保持不动。
        如果已在 ESTOP/RECOVERED 状态则保持。
        """
        self._estop_state = EStopState.ESTOP
        return RobotCommand(
            timestamp=timestamp,
            joint=JointState(q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            delta=JointState(q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        )

    def reset(self) -> None:
        """清除急停状态，允许恢复正常运行

        仅当状态为 RECOVERED 时复位到 NORMAL（需手动确认安全）。
        若仍为 ESTOP 状态，需先等待数据自动恢复（ESTOP→RECOVERED）。
        """
        if self._estop_state == EStopState.RECOVERED:
            self._estop_state = EStopState.NORMAL

    # ─── 配置热重载 ─────────────────────────────────

    def reload_config(self, cfg: ControlConfig) -> None:
        """运行时重载安全配置

        更新限位、速度阈值、频率等参数，不重置急停状态机。
        """
        self._cfg = cfg
        self._dt = 1.0 / cfg.frequency if cfg.frequency > 0 else 0.01

    # ─── 校验逻辑 ───────────────────────────────────

    def validate(self, command: RobotCommand) -> ControlResult:
        """校验 RobotCommand 并返回 ControlResult

        急停状态机逻辑：
        - ESTOP / RECOVERED → 仅产出零指令（不执行校验），等待复位
        - NORMAL → NaN/Inf 检测 → 限位检查 → 速度限幅
        """
        # ─── 急停态拦截器 ─────────────────────────────
        # ESTOP / RECOVERED 状态下，输出零指令
        # 若处于 ESTOP 且当前帧数据有效（无 NaN/Inf），自动进入 RECOVERED
        if self._estop_state != EStopState.NORMAL:
            if self._estop_state == EStopState.ESTOP:
                # 检查数据是否已恢复（所有关节值均为有限值）
                if all(math.isfinite(v) for v in command.joint.q):
                    self._estop_state = EStopState.RECOVERED
            return ControlResult(
                command=self._build_zero_command(command.timestamp),
                passed=False,
                violations=("E-stop active: command blocked",),
            )

        violations: list[str] = []
        q_list = list(command.joint.q)
        delta_list = list(command.delta.q)
        passed = True

        # 反推上一帧关节位置（用于限位/速度截断后同步更新 joint 和 delta）
        prev_q = tuple(
            command.joint.q[i] - command.delta.q[i]
            for i in range(6)
        )

        # 1. 异常值检测 (NaN / Inf)
        # 注意：必须优先于限位/速度检查，否则 NaN 会逃逸比较运算
        # (Python 中 float('nan') < 5.0 为 False，限位检查会静默放过 NaN)
        for val in q_list:
            if not math.isfinite(val):
                violations.append("Invalid value (NaN/Inf) detected - E-stop triggered")
                return ControlResult(
                    command=self.emergency_stop(timestamp=command.timestamp),
                    passed=False,
                    violations=tuple(violations),
                )

        # 2. 关节限位检查（截断后同步更新 delta）
        if self._cfg.enable_joint_limit:
            limits = [
                self._cfg.joint_limits.j1, self._cfg.joint_limits.j2,
                self._cfg.joint_limits.j3, self._cfg.joint_limits.j4,
                self._cfg.joint_limits.j5, self._cfg.joint_limits.j6,
            ]
            for i in range(6):
                min_l, max_l = limits[i]
                if q_list[i] < min_l:
                    q_list[i] = min_l
                    delta_list[i] = q_list[i] - prev_q[i]
                    violations.append(f"Joint {i+1} exceed min limit ({min_l})")
                    passed = False
                elif q_list[i] > max_l:
                    q_list[i] = max_l
                    delta_list[i] = q_list[i] - prev_q[i]
                    violations.append(f"Joint {i+1} exceed max limit ({max_l})")
                    passed = False

        # 3. 速度限幅检查与截断
        if self._cfg.enable_velocity_limit and self._dt > 0:
            for i in range(6):
                vel = abs(delta_list[i]) / self._dt
                if vel > self._cfg.max_joint_velocity:
                    violations.append(
                        f"Joint {i+1} exceeds max velocity "
                        f"(vel={vel:.3f} > {self._cfg.max_joint_velocity})"
                    )
                    passed = False
                    # 按比例缩减 delta，使速度降至 max_joint_velocity 以内
                    scale = self._cfg.max_joint_velocity / vel
                    delta_list[i] = delta_list[i] * scale
                    # 同步更新 joint
                    q_list[i] = prev_q[i] + delta_list[i]

        # 构造安全指令（使用截断后的 q 和 delta）
        safe_command = RobotCommand(
            timestamp=command.timestamp,
            joint=JointState(q=tuple(q_list)),
            delta=JointState(q=tuple(delta_list)),
        )

        return ControlResult(
            command=safe_command,
            passed=passed,
            violations=tuple(violations),
        )

    # ─── 内部辅助 ───────────────────────────────────

    @staticmethod
    def _build_zero_command(timestamp: float) -> RobotCommand:
        """构造零关节指令"""
        return RobotCommand(
            timestamp=timestamp,
            joint=JointState(q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            delta=JointState(q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        )