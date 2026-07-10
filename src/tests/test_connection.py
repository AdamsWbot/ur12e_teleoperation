# src/tests/test_connection.py
import pytest
import time
from src.common.config import (
    AppConfig,
    ControlConfig,
    FilterConfig,
    JointLimits,
    KeyboardConfig,
    MapperConfig,
    MasterConfig,
    S570Config,
    SlaveConfig,
    load_app_config,
)
from src.common.types import (
    RobotState,
    JointState,
    Pose,
    RawDeviceData,
)
from src.core.mapper import KeyboardMapper
from src.core.master import DefaultNormalizer


def _joint_limits() -> JointLimits:
    return JointLimits(
        j1=(-6.28, 6.28),
        j2=(-6.28, 6.28),
        j3=(-6.28, 6.28),
        j4=(-6.28, 6.28),
        j5=(-6.28, 6.28),
        j6=(-6.28, 6.28),
    )


def _keyboard_app_config() -> AppConfig:
    return AppConfig(
        device="keyboard",
        master=MasterConfig(
            ip="127.0.0.1",
            rtde_frequency=125,
            max_retries=1,
            retry_interval=0.0,
        ),
        s570=S570Config(usb_port="/dev/null"),
        keyboard=KeyboardConfig(joint_step=0.01),
        slave=SlaveConfig(
            ip="127.0.0.1",
            rtde_frequency=125,
            servoj_lookahead_time=0.03,
            servoj_gain=300,
            max_retries=1,
            retry_interval=0.0,
        ),
        control=ControlConfig(
            frequency=125,
            max_joint_velocity=1.2,
            enable_velocity_limit=True,
            enable_joint_limit=True,
            joint_limits=_joint_limits(),
        ),
        filter=FilterConfig(enable=True, alpha=0.2),
        mapper=MapperConfig(scale=1.0, joint_offset=(0, 0, 0, 0, 0, 0)),
    )

# ---------- 测试 1：纯逻辑（无硬件，任何时候都能跑） ----------
def test_normalizer_fills_missing_fields():
    """验证 DefaultNormalizer 对各种缺失数据的处理（计划 §2.3）"""
    norm = DefaultNormalizer()
    
    # 情况1：完全缺失
    raw_empty = RawDeviceData()
    state = norm.normalize(raw_empty)
    assert state.joint.q == (0.0,) * 6
    assert state.tcp_pose == Pose(0, 0, 0, 0, 0, 0)

    # 情况2：仅提供关节
    raw_joint = RawDeviceData(joint=(1, 2, 3, 4, 5, 6), tcp=None)
    state2 = norm.normalize(raw_joint)
    assert state2.joint.q == (1, 2, 3, 4, 5, 6)
    assert state2.tcp_pose == Pose(0, 0, 0, 0, 0, 0)
    assert abs(state2.timestamp - time.monotonic()) < 2.0

    # 情况3：仅提供 TCP
    raw_tcp = RawDeviceData(joint=None, tcp=Pose(1, 2, 3, 0, 0, 0))
    state3 = norm.normalize(raw_tcp)
    assert state3.joint.q == (0.0,) * 6
    assert state3.tcp_pose == Pose(1, 2, 3, 0, 0, 0)

    print("[PASS] Normalizer 缺字段补零逻辑全部通过")


def test_keyboard_mapper_outputs_absolute_command_and_delta():
    """KeyboardMapper 将键盘虚拟关节桥接为统一 RobotCommand。"""
    mapper = KeyboardMapper(_joint_limits())
    state1 = RobotState(
        timestamp=1.0,
        joint=JointState(q=(0.01, 0.0, 0.0, 0.0, 0.0, 0.0)),
        tcp_pose=Pose(0, 0, 0, 0, 0, 0),
    )
    cmd1 = mapper.map(state1, prev_command=None)

    assert cmd1.timestamp == 1.0
    assert cmd1.joint.q == state1.joint.q
    assert cmd1.delta.q == (0.0,) * 6

    state2 = RobotState(
        timestamp=2.0,
        joint=JointState(q=(0.02, -0.01, 0.0, 0.0, 0.0, 0.0)),
        tcp_pose=Pose(0, 0, 0, 0, 0, 0),
    )
    cmd2 = mapper.map(state2, prev_command=cmd1)

    assert cmd2.timestamp == 2.0
    assert cmd2.joint.q == state2.joint.q
    assert cmd2.delta.q == (0.01, -0.01, 0.0, 0.0, 0.0, 0.0)


def test_keyboard_pipeline_raw_to_command_without_hardware():
    """Keyboard RawDeviceData 可通过 normalizer + mapper 进入统一 pipeline。"""
    raw = RawDeviceData(joint=(0.0, 0.01, 0.0, 0.0, 0.0, 0.0), tcp=None)
    normalizer = DefaultNormalizer()
    mapper = KeyboardMapper(_joint_limits())

    state = normalizer.normalize(raw)
    command = mapper.map(state, prev_command=None)

    assert isinstance(state, RobotState)
    assert isinstance(command.joint, JointState)
    assert command.joint.q == (0.0, 0.01, 0.0, 0.0, 0.0, 0.0)
    assert command.delta.q == (0.0,) * 6


def test_factory_creates_keyboard_components_without_connecting_hardware():
    """Factory 可创建 keyboard device 与 KeyboardMapper，不触发硬件连接。"""
    from devices.keyboard import KeyboardReader
    from src.core.factory import SystemFactory

    factory = SystemFactory(_keyboard_app_config())

    assert isinstance(factory.create_device(), KeyboardReader)
    assert isinstance(factory.create_mapper(), KeyboardMapper)


# ---------- 测试 2：集成连接（需要真实硬件或 URSim） ----------
@pytest.mark.timeout(10)
def test_master_connection_and_read():
    """
    使用 P0 交付的 UR12eReader，验证完整链路：
    Reader.read() -> RawDeviceData -> DefaultNormalizer.normalize() -> RobotState
    所有断言自动检查类型、时间戳、数据完整性。
    """
    # 1. 加载配置
    try:
        cfg = load_app_config("config/config.yaml")
    except Exception as e:
        pytest.fail(f"无法加载 config/config.yaml: {e}")

    # 2. 通过 SystemFactory 创建设备（计划 §5.3：统一入口，禁止直接 import 设备类）
    try:
        from src.core.factory import SystemFactory
    except ImportError as e:
        pytest.skip(f"SystemFactory 不可用：{e}")

    factory = SystemFactory(cfg)
    reader = factory.create_device()
    normalizer = DefaultNormalizer()

    # 连接（内部自动重试）
    try:
        connected = reader.connect()
        if not connected:
            pytest.skip(f"主端设备不可达，跳过硬件连接测试：device={cfg.device}, ip={cfg.master.ip}")
        assert reader.is_connected, "is_connected 状态与 connect 返回值不一致"

        # 读取一帧原始数据
        raw = reader.read()
        assert raw.joint is not None and len(raw.joint) == 6, \
            "未从机械臂读到关节数据"

        # 标准化
        state = normalizer.normalize(raw)

        # ---------- 自动校验 ----------
        assert isinstance(state, RobotState)
        assert isinstance(state.joint, JointState)
        assert isinstance(state.tcp_pose, Pose)

        # 时间戳误差 < 2 秒
        now = time.monotonic()
        assert abs(state.timestamp - now) < 2.0, f"时间戳异常：{state.timestamp}"

        # 关节数量必须为6
        assert len(state.joint.q) == 6

        # TCP 位姿存在即可
        assert state.tcp_pose is not None

        # 可选：检查关节是否全零（仿真环境可能全零，实机一般不会）
        # 暂时注释，根据实际情况解注
        # assert any(abs(q) > 1e-6 for q in state.joint.q), "关节位置全零，机械臂可能未上电？"

        print(f"[PASS] RobotState: ts={state.timestamp:.3f}, "
              f"joint={state.joint.q[:3]}..., pose=({state.tcp_pose.x:.3f}, ...)")

    except Exception as e:
        pytest.fail(f"测试执行异常：{e}")

    finally:
        reader.disconnect()
        assert not reader.is_connected, "断开后 is_connected 仍为 True"
