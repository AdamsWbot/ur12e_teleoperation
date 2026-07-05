"""纯数据加载 — YAML → dataclass 转换，不含任何逻辑"""

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
