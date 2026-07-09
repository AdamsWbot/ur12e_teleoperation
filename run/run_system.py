#!/home/adams/ur12e_teleop_ws/venv/bin/python
"""完整遥操作 pipeline 主循环 — 计划1 §5.3

数据流:
  device.read() → RawDeviceData → normalize() → RobotState
  → map() → RobotCommand → validate() → ControlResult
  → filter() → RobotCommand → slave.execute()

启动方式:
  python run/run_system.py
"""

import socket
import sys
from pathlib import Path

# 确保项目根目录在 sys.path 中
_project_root = Path(__file__).resolve().parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.common.config import load_app_config
from src.core.control import SafetyController
from src.core.factory import SystemFactory
from src.core.filter import EMAFilter
from src.core.slave import SlaveController
from src.utils.logger import setup_logger
from src.utils.timer import Rate

logger = setup_logger("run_system")

# RTDE 端口 — 默认模式 (FLAG_UPLOAD_SCRIPT) 实际使用:
_RTDE_RECEIVE_PORT = 30004   # RTDEReceiveInterface — 实时数据流
_DASHBOARD_PORT = 29999       # Dashboard — 上传控制脚本


def _check_port(ip: str, port: int, timeout: float = 2.0) -> bool:
    """快速 TCP 端口可达性检查，2 秒内返回结果。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    result = sock.connect_ex((ip, port))
    sock.close()
    return result == 0


def main() -> None:
    # ── 1. 加载配置 ──────────────────────────────────
    cfg = load_app_config()
    logger.info("Config loaded: device=%s, frequency=%d Hz",
                cfg.device, cfg.control.frequency)

    # ── 2. 创建组件 ──────────────────────────────────
    factory = SystemFactory(cfg)

    device = factory.create_device()
    normalizer = factory.create_master()
    mapper = factory.create_mapper()
    slave = SlaveController(cfg.slave)
    safety = SafetyController(cfg.control)
    ema = EMAFilter(cfg.filter)
    rate = Rate(cfg.control.frequency)

    # ── 3. 连接设备 ──────────────────────────────────
    logger.info("正在连接主端设备 %s ...", cfg.master.ip)
    if not _check_port(cfg.master.ip, _RTDE_RECEIVE_PORT):
        logger.error(
            "主端设备 %s:%d 端口不可达，请确认 URSim/实机已启动",
            cfg.master.ip, _RTDE_RECEIVE_PORT,
        )
        return

    if not device.connect():
        logger.error("主端设备连接失败")
        return
    logger.info("主端设备已连接")

    logger.info("正在连接从臂 %s ...", cfg.slave.ip)
    if not _check_port(cfg.slave.ip, _DASHBOARD_PORT):
        logger.error(
            "从臂 %s:%d (Dashboard) 端口不可达，请确认实机已启动且在同一网络",
            cfg.slave.ip, _DASHBOARD_PORT,
        )
        device.disconnect()
        return

    if not slave.connect():
        logger.error("从臂连接失败")
        device.disconnect()
        return
    logger.info("从臂已连接")

    # ── 4. 主循环 ────────────────────────────────────
    prev_cmd = None
    logger.info("Pipeline 启动，按 Ctrl+C 退出")

    try:
        while True:
            # ① 读取主端原始数据
            raw = device.read()

            # ② 标准化为 RobotState
            state = normalizer.normalize(raw)

            # ③ 映射为 RobotCommand
            cmd = mapper.map(state, prev_cmd)

            # ④ 安全校验
            result = safety.validate(cmd)
            if not result.passed:
                logger.warning("安全违规: %s", result.violations)
                cmd = result.command  # 使用截断后的安全指令

            # ⑤ EMA 滤波平滑
            cmd = ema.apply(cmd)

            # ⑥ 发送到从臂
            ok = slave.execute(cmd)
            if not ok:
                logger.warning("servoJ 执行失败，继续下一帧")

            # ⑦ 保存上一帧指令（用于下帧 delta 计算）
            prev_cmd = cmd

            # ⑧ 按固定频率休眠
            rate.sleep()

    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，正在退出...")
    except Exception:
        logger.exception("Pipeline 异常退出")

    # ── 5. 清理 ──────────────────────────────────────
    slave.stop()
    slave.disconnect()
    device.disconnect()
    logger.info("Pipeline 已停止")


if __name__ == "__main__":
    main()
