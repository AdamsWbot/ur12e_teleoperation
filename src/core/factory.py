"""SystemFactory — 唯一允许跨模块 import 的类，负责组装全部组件。

计划2 §1.9: 完整 DI 容器，所有 create_* 返回的对象在程序生命周期内只创建一次。
"""

from devices.ur12e import UR12eReader
from devices.s570 import S570Reader
from devices.keyboard import KeyboardReader

from src.common.config import AppConfig
from src.core.master import DefaultNormalizer
from src.core.mapper import IdentityMapper, S570Mapper, KeyboardMapper
from src.core.control import SafetyController
from src.core.filter import EMAFilter

_VALID_DEVICES = frozenset({"ur12e", "s570", "keyboard"})


class SystemFactory:
    """将所有组件的创建集中在此，调用方不直接 import 实现类。

    用法:
        factory = SystemFactory(cfg)
        device    = factory.create_device()
        normalizer = factory.create_normalizer()
        mapper    = factory.create_mapper()
        controller = factory.create_controller()
        ema       = factory.create_filter()
        slave     = factory.create_slave()
    """

    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg

    # ── Device ─────────────────────────────────────

    def create_device(self):
        """根据 device 配置创建对应的主端 Reader。"""
        t = self.cfg.device
        if t == "ur12e":
            return UR12eReader(self.cfg.master)
        elif t == "s570":
            return S570Reader(self.cfg.s570)
        elif t == "keyboard":
            return KeyboardReader(self.cfg.keyboard)
        raise ValueError(
            f"Unknown device type: '{t}'，"
            f"必须为 {', '.join(sorted(_VALID_DEVICES))} 之一"
        )

    # ── Normalizer ─────────────────────────────────

    def create_normalizer(self):
        """创建数据标准化器 — RawDeviceData → RobotState。"""
        return DefaultNormalizer()

    def create_master(self):
        """已废弃 — 请使用 create_normalizer()。保留以兼容旧代码。"""
        return self.create_normalizer()

    # ── Mapper ─────────────────────────────────────

    def create_mapper(self):
        """根据 device 配置创建对应的 Mapper。

        TODO: S570Mapper 后续需接收 S570Config 以支持
        joint_mapping / joint_direction / enable_calibration 配置驱动映射。
        """
        t = self.cfg.device
        limits = self.cfg.control.joint_limits
        if t == "ur12e":
            return IdentityMapper(limits)
        elif t == "s570":
            return S570Mapper(limits)
        elif t == "keyboard":
            return KeyboardMapper(limits)
        raise ValueError(f"Unknown device type: '{t}'")

    # ── Safety Controller ──────────────────────────

    def create_controller(self):
        """创建安全控制器 — RobotCommand 校验（限位/速度/NaN）。"""
        return SafetyController(self.cfg.control)

    # ── Filter ─────────────────────────────────────

    def create_filter(self):
        """创建 EMA 滤波器 — 关节指令平滑。"""
        return EMAFilter(self.cfg.filter)

    # ── Slave ──────────────────────────────────────

    def create_slave(self):
        """创建从臂控制器 — RTDE servoJ 执行。

        ur_rtde 依赖仅在需要连实机/URSim 时才 import，
        纯开发/测试环境可不安装。
        """
        try:
            from src.core.slave import SlaveController
        except ImportError as exc:
            raise ImportError(
                "无法导入 SlaveController。请确认 ur_rtde 已安装: "
                "pip install ur_rtde==1.6.3"
            ) from exc
        return SlaveController(self.cfg.slave)
