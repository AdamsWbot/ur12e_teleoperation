import logging
import time
from dataclasses import dataclass, field
from typing import Optional


def setup_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """Create or return a logger using the project-wide format."""
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False

    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(
            "[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s"
        ))
        logger.addHandler(handler)

    for handler in logger.handlers:
        handler.setLevel(level)

    return logger


@dataclass
class PipelineMetrics:
    """Pipeline 单帧各阶段的耗时与状态"""
    read_time: float = 0.0          # device.read() 耗时（秒）
    normalize_time: float = 0.0     # normalizer.normalize() 耗时
    map_time: float = 0.0           # mapper.map() 耗时
    validate_time: float = 0.0      # controller.validate() 耗时
    filter_time: float = 0.0        # ema.apply() 耗时
    execute_time: float = 0.0       # slave.execute() 耗时
    total_time: float = 0.0         # 整帧总耗时

    violations: tuple[str, ...] = ()  # 安全违规信息
    has_error: bool = False          # 是否有异常
    frame_count: int = 0             # 帧序号


class PipelineLogger:
    """Pipeline 各阶段性能与状态的结构化日志记录器

    用法：
        plogger = PipelineLogger("system", level=logging.DEBUG)
        plogger.start_frame()
        # ... 执行各阶段 ...
        plogger.log_read(t)
        plogger.log_normalize(t)
        plogger.log_validation(result)
        plogger.end_frame()
    """

    def __init__(self, name: str = "pipeline", level: int = logging.INFO):
        self._logger = setup_logger(name, level)
        self._metrics = PipelineMetrics()
        self._frame_start: float = 0.0
        self._stage_start: float = 0.0
        self._frame_count = 0

    @property
    def logger(self) -> logging.Logger:
        """获取底层 logger，与 setup_logger() 兼容"""
        return self._logger

    @property
    def metrics(self) -> PipelineMetrics:
        """获取当前帧的 PipelineMetrics"""
        return self._metrics

    # ─── 帧生命周期 ─────────────────────────────────

    def start_frame(self) -> None:
        """开始新一帧的记录"""
        now = time.time()
        self._frame_start = now
        self._stage_start = now
        self._frame_count += 1
        self._metrics = PipelineMetrics(frame_count=self._frame_count)

    def end_frame(self) -> PipelineMetrics:
        """结束当前帧，记录总耗时，返回本帧指标"""
        self._metrics.total_time = time.time() - self._frame_start
        if self._logger.isEnabledFor(logging.DEBUG):
            m = self._metrics
            self._logger.debug(
                f"Frame {m.frame_count}: "
                f"read={m.read_time*1000:.1f}ms "
                f"norm={m.normalize_time*1000:.1f}ms "
                f"map={m.map_time*1000:.1f}ms "
                f"valid={m.validate_time*1000:.1f}ms "
                f"filter={m.filter_time*1000:.1f}ms "
                f"exec={m.execute_time*1000:.1f}ms "
                f"total={m.total_time*1000:.1f}ms"
            )
        return self._metrics

    # ─── 各阶段计时 ─────────────────────────────────

    def log_read(self, elapsed: float) -> None:
        """记录 device.read() 耗时"""
        self._metrics.read_time = elapsed

    def log_normalize(self, elapsed: float) -> None:
        """记录 normalizer.normalize() 耗时"""
        self._metrics.normalize_time = elapsed

    def log_map(self, elapsed: float) -> None:
        """记录 mapper.map() 耗时"""
        self._metrics.map_time = elapsed

    def log_validation(self, result) -> None:
        """记录 controller.validate() 耗时和违规信息"""
        self._metrics.validate_time = time.time() - self._stage_start
        self._metrics.violations = result.violations
        self._metrics.has_error = not result.passed

        if result.violations and self._logger.isEnabledFor(logging.WARNING):
            self._logger.warning(
                f"Frame {self._frame_count}: safety violations: {result.violations}"
            )

    def log_filter(self, elapsed: float) -> None:
        """记录 ema.apply() 耗时"""
        self._metrics.filter_time = elapsed

    def log_execute(self, elapsed: float) -> None:
        """记录 slave.execute() 耗时"""
        self._metrics.execute_time = elapsed

    # ─── 阶段计时辅助 ───────────────────────────────

    def mark_stage(self) -> None:
        """标记当前阶段结束（用于自动计算本阶段耗时）"""
        self._stage_start = time.time()

    def elapsed_since_stage(self) -> float:
        """返回从上次 mark_stage() 到现在的秒数"""
        return time.time() - self._stage_start