"""纯数据加载 — YAML → dataclass 转换，不含任何逻辑"""

import re
from dataclasses import dataclass

import yaml


# ─── 主端配置 ──────────────────────────────────────────

@dataclass
class MasterConfig:
    ip: str
    rtde_frequency: int
    max_retries: int
    retry_interval: float


@dataclass
class S570Config:
    usb_port: str
    active_arm: str = "left"                       # [计划2] "left" | "right"
    enable_calibration: bool = True                # [计划2] 启动时自动零点校准
    joint_mapping: tuple[int, ...] = (1, 2, 3, 4, 5, 6)  # [计划2] S570→UR 关节索引 (1-based, 6个)
    joint_direction: tuple[int, ...] = (1, 1, 1, 1, 1, 1) # [计划2] 方向 ±1


@dataclass
class KeyboardConfig:
    joint_step: float


# ─── 从端配置 ──────────────────────────────────────────

@dataclass
class SlaveConfig:
    ip: str
    rtde_frequency: int
    servoj_lookahead_time: float
    servoj_gain: int
    max_retries: int
    retry_interval: float


# ─── 控制配置 ──────────────────────────────────────────

@dataclass
class JointLimits:
    j1: tuple[float, float]
    j2: tuple[float, float]
    j3: tuple[float, float]
    j4: tuple[float, float]
    j5: tuple[float, float]
    j6: tuple[float, float]


@dataclass
class ControlConfig:
    frequency: int
    max_joint_velocity: float
    enable_velocity_limit: bool
    enable_joint_limit: bool
    joint_limits: JointLimits


# ─── 滤波 / 映射配置 ───────────────────────────────────

@dataclass
class FilterConfig:
    enable: bool
    alpha: float


@dataclass
class MapperConfig:
    scale: float
    joint_offset: tuple[float, ...]


# ─── 顶层配置 ──────────────────────────────────────────

@dataclass
class AppConfig:
    device: str
    master: MasterConfig
    s570: S570Config
    keyboard: KeyboardConfig
    slave: SlaveConfig
    control: ControlConfig
    filter: FilterConfig
    mapper: MapperConfig


# ─── 常量 ─────────────────────────────────────────────

_VALID_DEVICES = frozenset({"ur12e", "s570", "keyboard"})
_IP_PATTERN = re.compile(
    r"^(?:(?:25[0-5]|2[0-4]\d|[01]?\d\d?)\.){3}"
    r"(?:25[0-5]|2[0-4]\d|[01]?\d\d?)$"
)
_JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")


# ─── 校验 ─────────────────────────────────────────────

def validate_config(cfg: AppConfig) -> list[str]:
    """校验 AppConfig 完整性，返回警告/错误列表。空列表 = 全部通过。

    校验规则:
      - device 必须为 ur12e / s570 / keyboard 之一
      - device=ur12e 时 master.ip 格式合法（IPv4）
      - device=s570 时 s570.usb_port 非空
      - device=keyboard 时 keyboard.joint_step > 0
      - slave.ip 非空
      - control.frequency > 0
      - control.max_joint_velocity >= 0
      - JointLimits 每关节 min < max
      - filter.alpha 在 (0, 1]
      - mapper.scale > 0
    """
    warnings: list[str] = []

    # ── device ──────────────────────────────────────
    if cfg.device not in _VALID_DEVICES:
        warnings.append(
            f"device 必须为 {', '.join(sorted(_VALID_DEVICES))} 之一，当前: '{cfg.device}'"
        )

    # ── master ──────────────────────────────────────
    if cfg.device == "ur12e":
        if not cfg.master.ip.strip():
            warnings.append("device=ur12e 时 master.ip 不能为空")
        elif not _IP_PATTERN.match(cfg.master.ip.strip()):
            warnings.append(f"master.ip 格式不合法: '{cfg.master.ip}'")

    # ── s570 ────────────────────────────────────────
    if cfg.device == "s570":
        if not cfg.s570.usb_port.strip():
            warnings.append("device=s570 时 s570.usb_port 不能为空")
        if cfg.s570.active_arm not in ("left", "right"):
            warnings.append(
                f"s570.active_arm 必须为 left 或 right，当前: '{cfg.s570.active_arm}'"
            )
        _validate_s570_mapping(cfg, warnings)
        _validate_s570_direction(cfg, warnings)

    # ── keyboard ────────────────────────────────────
    if cfg.device == "keyboard":
        if cfg.keyboard.joint_step <= 0:
            warnings.append(
                f"keyboard.joint_step 必须 > 0，当前: {cfg.keyboard.joint_step}"
            )

    # ── slave ───────────────────────────────────────
    if not cfg.slave.ip.strip():
        warnings.append("slave.ip 不能为空")

    # ── control ─────────────────────────────────────
    if cfg.control.frequency <= 0:
        warnings.append(f"control.frequency 必须 > 0，当前: {cfg.control.frequency}")
    if cfg.control.max_joint_velocity < 0:
        warnings.append(
            f"control.max_joint_velocity 必须 >= 0，当前: {cfg.control.max_joint_velocity}"
        )

    limits = cfg.control.joint_limits
    for jn in _JOINT_NAMES:
        lo, hi = getattr(limits, jn)
        if lo >= hi:
            warnings.append(f"joint_limits.{jn}: min ({lo}) 必须 < max ({hi})")

    # ── filter ──────────────────────────────────────
    if not (0.0 < cfg.filter.alpha <= 1.0):
        warnings.append(
            f"filter.alpha 必须在 (0, 1] 范围内，当前: {cfg.filter.alpha}"
        )

    # ── mapper ──────────────────────────────────────
    if cfg.mapper.scale <= 0:
        warnings.append(f"mapper.scale 必须 > 0，当前: {cfg.mapper.scale}")

    return warnings


def _validate_s570_mapping(cfg: AppConfig, warnings: list[str]) -> None:
    """校验 s570.joint_mapping: 长度=6, 值∈{1..7}, 无重复。"""
    mapping = cfg.s570.joint_mapping
    if len(mapping) != 6:
        warnings.append(
            f"s570.joint_mapping 需要恰好 6 个元素，当前: {len(mapping)}"
        )
        return
    for val in mapping:
        if val not in range(1, 7):
            warnings.append(
                f"s570.joint_mapping 每个值必须在 1–6 范围内，当前含: {val}"
            )
            break
    if len(set(mapping)) != len(mapping):
        warnings.append(f"s570.joint_mapping 包含重复值: {mapping}")


def _validate_s570_direction(cfg: AppConfig, warnings: list[str]) -> None:
    """校验 s570.joint_direction: 长度=6, 每个值 ∈ {1, -1}。"""
    direction = cfg.s570.joint_direction
    if len(direction) != 6:
        warnings.append(
            f"s570.joint_direction 需要恰好 6 个元素，当前: {len(direction)}"
        )
        return
    for i, val in enumerate(direction):
        if val not in (1, -1):
            warnings.append(
                f"s570.joint_direction[{i}] 必须为 1 或 -1，当前: {val}"
            )
            break


# ─── 加载函数 ──────────────────────────────────────────

def load_app_config(path: str = "config/config.yaml") -> AppConfig:
    """从 config.yaml 加载并解析为 AppConfig"""
    with open(path) as f:
        data = yaml.safe_load(f)

    control = data["control"]
    joint_limits = JointLimits(**control.pop("joint_limits"))

    return AppConfig(
        device=data["device"],
        master=MasterConfig(**data["master"]),
        s570=S570Config(**data["s570"]),
        keyboard=KeyboardConfig(**data["keyboard"]),
        slave=SlaveConfig(**data["slave"]),
        control=ControlConfig(joint_limits=joint_limits, **control),
        filter=FilterConfig(**data["filter"]),
        mapper=MapperConfig(**data["mapper"]),
    )


def load_and_validate(path: str = "config/config.yaml") -> tuple[AppConfig, list[str]]:
    """加载配置并校验，返回 (AppConfig, 警告列表)。

    警告列表为空表示配置完全通过校验。
    """
    cfg = load_app_config(path)
    warnings = validate_config(cfg)
    return cfg, warnings
