#!/home/adams/ur12e_teleop_ws/venv/bin/python
"""从臂独立测试脚本 — 计划1 §2.11, 计划2 §1.12

用于在无主端设备的情况下独立验证从臂 RTDE 通信和 servoJ 控制。

模式:
  --hold    保持当前位置（验证连接+servoJ 通路）
  --test    运行预设小幅度正弦测试序列（验证关节跟踪）

示例:
  python run/run_slave.py --hold                     # 保持当前位置
  python run/run_slave.py --test --duration 10       # 10 秒正弦测试
  python run/run_slave.py --test --joint 5 --gain 200  # 调参测试
"""

import argparse
import math
import sys
import time
from pathlib import Path

_project_root = Path(__file__).resolve().parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.common.config import SlaveConfig, load_app_config
from src.common.types import JointState, RobotCommand
from src.core.slave import SlaveController
from src.utils.logger import setup_logger
from src.utils.timer import Rate

logger = setup_logger("run_slave")


def _build_cfg(args: argparse.Namespace) -> SlaveConfig:
    """用命令行参数覆盖配置文件中的从臂参数。"""
    cfg = load_app_config()
    return SlaveConfig(
        ip=cfg.slave.ip,
        rtde_frequency=args.frequency or cfg.slave.rtde_frequency,
        servoj_lookahead_time=args.lookahead or cfg.slave.servoj_lookahead_time,
        servoj_gain=args.gain or cfg.slave.servoj_gain,
        max_retries=cfg.slave.max_retries,
        retry_interval=cfg.slave.retry_interval,
    )


# ── hold 模式 ──────────────────────────────────────

def run_hold(ctrl: SlaveController, rate: Rate) -> None:
    """读取当前位置并持续发送 servoJ 保持不动。"""
    state = ctrl.get_state()
    if state is None:
        logger.error("无法读取从臂当前关节位置，放弃 hold")
        return

    hold_q = list(state.joint.q)
    logger.info("Hold 位置: %s", [f"{q:.4f}" for q in hold_q])
    logger.info("按 Ctrl+C 停止")

    while True:
        cmd = RobotCommand(
            timestamp=time.monotonic(),
            joint=JointState(q=tuple(hold_q)),
            delta=JointState(q=(0.0,) * 6),
        )
        ctrl.execute(cmd)
        rate.sleep()


# ── test 模式 ──────────────────────────────────────

def run_test(ctrl: SlaveController, rate: Rate, args: argparse.Namespace) -> None:
    """在指定关节上运行小幅度正弦波测试序列。"""
    state = ctrl.get_state()
    if state is None:
        logger.error("无法读取从臂当前关节位置，放弃 test")
        return

    base_q = list(state.joint.q)
    joint_idx = args.joint - 1  # 用户输入 1-6，转为 0-5
    amplitude = args.amplitude
    period = args.period
    duration = args.duration
    dt = 1.0 / args.frequency if args.frequency else 1.0 / 125

    logger.info(
        "测试关节 J%d, 振幅 %.3f rad, 周期 %.1f s, 持续 %.1f s",
        args.joint, amplitude, period, duration,
    )
    logger.info("起始位置: %s", [f"{q:.4f}" for q in base_q])
    logger.info("按 Ctrl+C 提前停止")

    start_time = time.monotonic()
    while True:
        elapsed = time.monotonic() - start_time
        if elapsed > duration:
            logger.info("测试时长结束，回到起始位置")
            break

        # 正弦波偏移
        offset = amplitude * math.sin(2 * math.pi * elapsed / period)
        q = list(base_q)
        q[joint_idx] += offset

        cmd = RobotCommand(
            timestamp=time.monotonic(),
            joint=JointState(q=tuple(q)),
            delta=JointState(q=(0.0,) * 6),
        )
        ctrl.execute(cmd)
        rate.sleep()

    # 回到起始位置
    logger.info("正在回到起始位置...")
    for _ in range(50):  # 约 0.4 秒过渡
        cmd = RobotCommand(
            timestamp=time.monotonic(),
            joint=JointState(q=tuple(base_q)),
            delta=JointState(q=(0.0,) * 6),
        )
        ctrl.execute(cmd)
        rate.sleep()


# ── 主入口 ─────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="UR12e 从臂独立测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s --hold               保持当前位置
  %(prog)s --test               在 J6 上运行默认正弦测试
  %(prog)s --test --joint 5 --amplitude 0.1 --duration 15
        """,
    )

    # 模式（二选一）
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--hold", action="store_true",
                      help="保持当前关节位置不动")
    mode.add_argument("--test", action="store_true",
                      help="运行预设正弦测试序列")

    # 测试参数
    parser.add_argument("--joint", type=int, default=6, choices=range(1, 7),
                        help="测试目标关节 1-6 (默认: 6, 腕关节)")
    parser.add_argument("--amplitude", type=float, default=0.05,
                        help="正弦振幅 [rad] (默认: 0.05)")
    parser.add_argument("--period", type=float, default=4.0,
                        help="正弦周期 [s] (默认: 4.0)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="测试持续时长 [s] (默认: 10.0)")

    # 覆盖配置参数
    parser.add_argument("--lookahead", type=float, default=None,
                        help="覆盖 servoj_lookahead_time")
    parser.add_argument("--gain", type=int, default=None,
                        help="覆盖 servoj_gain")
    parser.add_argument("--frequency", type=int, default=None,
                        help="覆盖 rtde_frequency")

    args = parser.parse_args()

    # ── 构建配置 ──────────────────────────────────
    slave_cfg = _build_cfg(args)
    ctrl = SlaveController(slave_cfg)
    rate = Rate(slave_cfg.rtde_frequency)

    logger.info("从臂 IP: %s, frequency: %d Hz, lookahead: %.3f, gain: %d",
                slave_cfg.ip, slave_cfg.rtde_frequency,
                slave_cfg.servoj_lookahead_time, slave_cfg.servoj_gain)

    # ── 连接 ──────────────────────────────────────
    logger.info("正在连接从臂 %s ...", slave_cfg.ip)
    if not ctrl.connect():
        logger.error("从臂连接失败")
        return
    logger.info("从臂已连接")

    # ── 执行模式 ──────────────────────────────────
    try:
        if args.hold:
            run_hold(ctrl, rate)
        else:
            run_test(ctrl, rate, args)
    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，正在退出...")
    except Exception:
        logger.exception("运行异常")

    # ── 清理 ──────────────────────────────────────
    ctrl.stop()
    ctrl.disconnect()
    logger.info("从臂测试结束")


if __name__ == "__main__":
    main()
