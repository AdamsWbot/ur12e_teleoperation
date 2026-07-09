#!/home/adams/ur12e_teleop_ws/venv/bin/python
"""主端设备独立调试工具 — 计划2 §4.8

只读取并打印 RawDeviceData / RobotState，不连接从臂。
用于验证主端设备（UR12e / S570 / Keyboard）的数据采集通路。

启动方式:
  python run/run_master.py                 # 使用 config.yaml 中的 device
  python run/run_master.py --device s570   # 覆盖设备类型
  python run/run_master.py --raw           # 仅打印 RawDeviceData（跳过 normalizer）
"""

import argparse
import sys
import time
from pathlib import Path

_project_root = Path(__file__).resolve().parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.common.config import load_app_config
from src.core.factory import SystemFactory
from src.utils.logger import setup_logger
from src.utils.timer import Rate

logger = setup_logger("run_master")


def _print_raw(raw, i: int) -> None:
    """打印 RawDeviceData 字段。"""
    joint_str = [f"{v:.4f}" for v in raw.joint] if raw.joint else "None"
    sec_str = [f"{v:.4f}" for v in raw.joint_secondary] if raw.joint_secondary else "None"
    tcp_str = "None"
    if raw.tcp:
        tcp_str = (f"({raw.tcp.x:.4f},{raw.tcp.y:.4f},{raw.tcp.z:.4f},"
                   f"{raw.tcp.rx:.4f},{raw.tcp.ry:.4f},{raw.tcp.rz:.4f})")
    print(f"[{i:04d}] joint={joint_str}")
    print(f"      joint_secondary={sec_str}")
    print(f"      tcp={tcp_str}")
    print(f"      buttons={raw.buttons:#010b}, axes={raw.axes}")


def _print_state(state, i: int) -> None:
    """打印 RobotState 字段。"""
    q = [f"{v:.4f}" for v in state.joint.q]
    tcp = (f"({state.tcp_pose.x:.4f},{state.tcp_pose.y:.4f},{state.tcp_pose.z:.4f},"
           f"{state.tcp_pose.rx:.4f},{state.tcp_pose.ry:.4f},{state.tcp_pose.rz:.4f})")
    print(f"[{i:04d}] ts={state.timestamp:.3f}  joint={q}  tcp={tcp}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="UR12e 主端设备调试工具 — 只读取、打印，不连从臂",
    )
    parser.add_argument("--device", type=str, default=None,
                        help="覆盖 config.yaml 中的 device 类型")
    parser.add_argument("--raw", action="store_true",
                        help="仅打印 RawDeviceData（跳过 normalizer）")
    parser.add_argument("--rate", type=int, default=10,
                        help="打印频率 [Hz] (默认: 10)")
    args = parser.parse_args()

    # ── 加载配置 ──────────────────────────────────
    cfg = load_app_config()
    if args.device:
        cfg.device = args.device
        logger.info("设备类型已覆写为: %s", args.device)

    # ── 创建组件 ──────────────────────────────────
    factory = SystemFactory(cfg)
    device = factory.create_device()
    normalizer = factory.create_normalizer()
    rate = Rate(args.rate)

    # ── 连接 ──────────────────────────────────────
    logger.info("正在连接 %s ...", cfg.device)
    if not device.connect():
        logger.error("主端设备连接失败")
        return
    logger.info("主端设备已连接，按 Ctrl+C 退出")

    # ── 读取循环 ──────────────────────────────────
    i = 0
    try:
        while True:
            raw = device.read()
            if args.raw:
                _print_raw(raw, i)
            else:
                state = normalizer.normalize(raw)
                _print_state(state, i)
            i += 1
            rate.sleep()
    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，正在退出...")
    except Exception:
        logger.exception("读取异常")
    finally:
        device.disconnect()
        logger.info("主端调试工具已退出")


if __name__ == "__main__":
    main()
