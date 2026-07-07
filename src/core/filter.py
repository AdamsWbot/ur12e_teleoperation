from src.common.config import FilterConfig
from src.common.types import JointState, RobotCommand


class EMAFilter:
    """Exponential moving average filter for joint commands."""

    def __init__(self, cfg: FilterConfig) -> None:
        if not 0.0 < cfg.alpha <= 1.0:
            raise ValueError(f"filter alpha must be in (0, 1], got {cfg.alpha}")
        self._enabled = cfg.enable
        self._alpha = cfg.alpha
        self._last_joint: JointState | None = None

    def apply(self, command: RobotCommand) -> RobotCommand:
        if not self._enabled:
            return command

        if self._last_joint is None:
            self._last_joint = command.joint
            return command

        smoothed_q = tuple(
            self._alpha * current + (1.0 - self._alpha) * previous
            for current, previous in zip(command.joint.q, self._last_joint.q)
        )
        smoothed_joint = JointState(q=smoothed_q)
        self._last_joint = smoothed_joint

        return RobotCommand(
            timestamp=command.timestamp,
            joint=smoothed_joint,
            delta=command.delta,
        )

    def reset(self) -> None:
        self._last_joint = None
