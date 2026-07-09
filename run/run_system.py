#!/home/adams/ur12e_teleop_ws/venv/bin/python
"""完整遥操作 pipeline 主循环 — 计划2 §4.7

数据流:
  device.read() → RawDeviceData → normalize() → RobotState
  → map() → RobotCommand → validate() → ControlResult
  → filter() → RobotCommand → slave.execute()

启动方式:
  python run/run_system.py
"""

import sys
from pathlib import Path

# 确保项目根目录在 sys.path 中
_project_root = Path(__file__).resolve().parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.common.config import load_and_validate
from src.core.factory import SystemFactory
from src.utils.logger import setup_logger
from src.utils.timer import Rate

logger = setup_logger("run_system")


def main() -> None:
    # ── 1. 加载 + 校验配置 ────────────────────────────
    cfg, warnings = load_and_validate()
    for w in warnings:
        logger.warning("配置警告: %s", w)
    logger.info("Config loaded: device=%s, frequency=%d Hz",
                cfg.device, cfg.control.frequency)

    # ── 2. 创建全部组件（唯一跨模块 import 点）──────────
    factory = SystemFactory(cfg)

    device     = factory.create_device()
    normalizer = factory.create_normalizer()
    mapper     = factory.create_mapper()
    controller = factory.create_controller()
    ema        = factory.create_filter()
    slave      = factory.create_slave()
    rate       = Rate(cfg.control.frequency)

    # ── 3. 连接设备 ────────────────────────────────────
    logger.info("正在连接主端设备 %s ...", cfg.master.ip)
    if not device.connect():
        logger.error("主端设备连接失败")
        return
    logger.info("主端设备已连接")

    logger.info("正在连接从臂 %s ...", cfg.slave.ip)
    if not slave.connect():
        logger.error("从臂连接失败")
        device.disconnect()
        return
    logger.info("从臂已连接")

    # ── 4. S570 零点校准（第一帧）─────────────────────────
    if cfg.device == "s570" and cfg.s570.enable_calibration:
        raw = device.read()
        state = normalizer.normalize(raw)
        # calibrate() 内部完成 7→6 映射后再记录 6 关节零点偏移
        mapper.calibrate(state.joint)
        logger.info("S570 校准完成: %s",
                    [f"{v:.3f}" for v in state.joint.q])

    # ── 5. 主循环 ──────────────────────────────────────
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
            result = controller.validate(cmd)
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

    # ── 6. 清理 ────────────────────────────────────────
    slave.stop()
    slave.disconnect()
    device.disconnect()
    logger.info("Pipeline 已停止")


if __name__ == "__main__":
    main()
