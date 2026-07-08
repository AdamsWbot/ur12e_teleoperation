"""从臂控制执行 — 通过 RTDE 向 UR12e 从臂发送 servoj 指令

计划1 §2.4: 输入 RobotCommand，输出 RTDE 控制指令到从臂。
计划1 §3.4 接口合约: 输入 RobotCommand → 输出 bool，异常为 ConnectionError / RuntimeError。

RTDE 端口概览 (ur_rtde 1.6.3):
  - RTDEControlInterface  → 50002 (URCap External Control)
  - RTDEReceiveInterface   → 30004 (RTDE data stream)
"""

import os
import socket
import signal
import time
import logging

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

from src.common.config import SlaveConfig
from src.common.types import JointState, Pose, RobotCommand, RobotState

logger = logging.getLogger(__name__)

# 单次 RTDE 连接尝试的超时时间（秒），会向上取整（signal.alarm 只接受整数）
_CONNECT_ATTEMPT_TIMEOUT = 5

# RTDE 端口
_CONTROL_PORT = 50002   # RTDEControlInterface (URCap External Control)
_RECEIVE_PORT = 30004   # RTDEReceiveInterface (RTDE data)


class _ConnectTimeout(Exception):
    """signal.alarm 触发的连接超时异常。"""


def _on_connect_timeout(signum, frame):
    raise _ConnectTimeout


class SlaveController:
    """通过 RTDE 向 UR12e 从臂发送 servoj 控制指令。

    同时持有两条 RTDE 连接：
    - RTDEControlInterface (port 50002): 发送 servoj / stopScript
    - RTDEReceiveInterface (port 30004): 读取从臂实际状态 (get_state)

    连接使用 signal.alarm 超时机制，避免目标不可达时 TCP 长时间阻塞。
    """

    def __init__(self, cfg: SlaveConfig) -> None:
        self._ip = cfg.ip
        self._frequency = cfg.rtde_frequency
        self._lookahead = cfg.servoj_lookahead_time
        self._gain = cfg.servoj_gain
        self._max_retries = cfg.max_retries
        self._retry_interval = cfg.retry_interval
        # 控制周期 (dt)，用于 servoJ 的 time 参数
        self._dt = 1.0 / cfg.rtde_frequency if cfg.rtde_frequency > 0 else 0.008

        self._control: RTDEControlInterface | None = None
        self._receive: RTDEReceiveInterface | None = None

    # ── 连接管理 ──────────────────────────────────

    def connect(self) -> bool:
        """连接从臂 RTDE 控制与接收接口，失败自动重试。

        每次尝试最多 {_CONNECT_ATTEMPT_TIMEOUT} 秒（通过 signal.alarm 中断阻塞的 C 调用）。

        Returns:
            True 如果两条连接均建立成功。
        """
        # ── 0. 快速端口诊断 ──────────────────────────
        self._diagnose_ports()

        old_handler = signal.signal(signal.SIGALRM, _on_connect_timeout)

        for attempt in range(1, self._max_retries + 1):
            logger.info("Slave connecting to %s (attempt %d/%d)...",
                        self._ip, attempt, self._max_retries)

            # 分别尝试 control 和 receive，明确报告哪一步失败
            ctrl_ok = False
            recv_ok = False

            try:
                logger.info("  → 尝试连接 RTDEControlInterface (port %d) ...", _CONTROL_PORT)
                self._control = self._timed_call(
                    RTDEControlInterface, self._ip,
                )
                if self._control is not None and self._control.isConnected():
                    ctrl_ok = True
                    logger.info("  ✓ RTDEControlInterface 已连接")
                else:
                    logger.warning("  ✗ RTDEControlInterface 连接返回 None 或未连接")
            except _ConnectTimeout:
                logger.warning("  ✗ RTDEControlInterface 连接超时 (%ds)", _CONNECT_ATTEMPT_TIMEOUT)

            try:
                logger.info("  → 尝试连接 RTDEReceiveInterface (port %d, freq=%d Hz) ...",
                           _RECEIVE_PORT, self._frequency)
                self._receive = self._timed_call(
                    RTDEReceiveInterface, self._ip, self._frequency,
                )
                if self._receive is not None and self._receive.isConnected():
                    recv_ok = True
                    logger.info("  ✓ RTDEReceiveInterface 已连接")
                else:
                    logger.warning("  ✗ RTDEReceiveInterface 连接返回 None 或未连接")
            except _ConnectTimeout:
                logger.warning("  ✗ RTDEReceiveInterface 连接超时 (%ds)", _CONNECT_ATTEMPT_TIMEOUT)

            if ctrl_ok and recv_ok:
                logger.info("Slave connected to %s", self._ip)
                signal.signal(signal.SIGALRM, old_handler)
                return True

            # 报告本次尝试失败原因摘要
            if not ctrl_ok and not recv_ok:
                logger.warning("  尝试 %d 失败: 两条连接均未建立", attempt)
            elif not ctrl_ok:
                logger.warning("  尝试 %d 失败: Control 口未连通 (port %d)", attempt, _CONTROL_PORT)
            else:
                logger.warning("  尝试 %d 失败: Receive 口未连通 (port %d)", attempt, _RECEIVE_PORT)

            self._cleanup()
            if attempt < self._max_retries:
                time.sleep(self._retry_interval)

        signal.signal(signal.SIGALRM, old_handler)
        logger.error(
            "Slave failed to connect to %s after %d attempts",
            self._ip, self._max_retries,
        )
        return False

    def _diagnose_ports(self) -> None:
        """快速 TCP 端口可达性检查（2 秒超时），在连接前输出诊断信息。"""
        logger.info("─── 端口诊断 %s ───", self._ip)
        for port, name in [(_CONTROL_PORT, "RTDE Control"),
                           (_RECEIVE_PORT, "RTDE Receive")]:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            result = sock.connect_ex((self._ip, port))
            sock.close()
            if result == 0:
                logger.info("  ✓ %s port %d — 可达", name, port)
            else:
                logger.warning("  ✗ %s port %d — 不可达 (errno=%d)", name, port, result)
        logger.info("─── 诊断结束 ───")

    @staticmethod
    def _timed_call(factory, *args, timeout=_CONNECT_ATTEMPT_TIMEOUT):
        """调用 factory(*args)，超时通过 signal.alarm 中断。

        signal.alarm 通过系统调用中断来工作，因此可以中断 ur_rtde
        内部阻塞的 C socket 连接。

        注意: 第一次尝试不抑制 stderr，以便看到 ur_rtde C 层的错误信息；
        后续尝试抑制 stderr 以减少噪音。
        """
        signal.alarm(timeout)
        try:
            return factory(*args)
        except _ConnectTimeout:
            logger.warning("  ⏱ 连接超时 (%ds) — 可能原因: 防火墙、URCap 未运行、网络不通", timeout)
            return None
        except Exception as exc:
            logger.warning("  ✗ 连接异常: %s: %s", type(exc).__name__, exc)
            return None
        finally:
            signal.alarm(0)

    def disconnect(self) -> None:
        """断开从臂所有 RTDE 连接。"""
        logger.info("Slave disconnecting")
        self._cleanup()

    def _cleanup(self) -> None:
        """安全释放 RTDE 接口资源。"""
        if self._control is not None:
            try:
                self._control.disconnect()
            except Exception:
                pass
            self._control = None
        if self._receive is not None:
            try:
                self._receive.disconnect()
            except Exception:
                pass
            self._receive = None

    # ── 指令执行 ──────────────────────────────────

    def execute(self, command: RobotCommand) -> bool:
        """发送 servoj 关节位置指令到从臂。

        servoJ signature (ur_rtde 1.6.3):
            servoJ(q, speed, acc, time, lookahead_time, gain) -> bool
            - speed / acc: NOT used in current version
            - time: 指令控制时长 [s]，函数阻塞 time 秒

        Args:
            command: 包含目标关节位置 (joint.q) 的 RobotCommand。

        Returns:
            True 如果指令成功发送。
        """
        if self._control is None:
            logger.warning("execute() called before connect()")
            return False
        try:
            self._control.servoJ(
                list(command.joint.q),
                0.0,             # speed — NOT used in current ur_rtde
                0.0,             # acceleration — NOT used in current ur_rtde
                self._dt,        # time — 控制周期 (1/frequency)
                self._lookahead,
                self._gain,
            )
            return True
        except Exception:
            logger.warning("servoJ failed", exc_info=True)
            return False

    def stop(self) -> None:
        """紧急停止从臂 — 调用 stopScript 中断当前运行脚本。"""
        if self._control is not None:
            try:
                self._control.stopScript()
                logger.info("Slave emergency stop executed")
            except Exception:
                logger.error("Slave stopScript failed", exc_info=True)

    # ── 状态读取 ──────────────────────────────────

    def get_state(self) -> RobotState | None:
        """读取从臂当前关节位置与 TCP 位姿。

        用于调试/日志/状态监控；当前阶段允许返回 None。

        Returns:
            RobotState 如果读取成功，None 如果 receive 接口不可用或读取失败。
        """
        if self._receive is None:
            return None
        try:
            actual_q = self._receive.getActualQ()
            tcp = self._receive.getActualTCPPose()
            return RobotState(
                timestamp=time.monotonic(),
                joint=JointState(q=tuple(actual_q)),
                tcp_pose=Pose(
                    x=tcp[0], y=tcp[1], z=tcp[2],
                    rx=tcp[3], ry=tcp[4], rz=tcp[5],
                ),
            )
        except Exception:
            logger.warning("get_state failed", exc_info=True)
            return None

    @property
    def is_connected(self) -> bool:
        return (
            self._control is not None
            and self._control.isConnected()
            and self._receive is not None
            and self._receive.isConnected()
        )
