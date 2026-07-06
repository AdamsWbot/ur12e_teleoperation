# src/tests/test_connection.py
import pytest
import time
from src.common.config import load_app_config
from src.common.types import (
    RobotState,
    JointState,
    Pose,
    MasterReader,
)
from src.core.master import DefaultNormalizer

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


# ---------- 测试 2：集成连接（需要真实硬件或 URSim） ----------
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

    # 2. 导入并使用真实的 UR12eReader（由 P0 完成，不允许修改）
    try:
        from src.devices.ur12e import UR12eReader
    except ImportError as e:
        pytest.skip(f"真实 UR12eReader 不可用（可能缺少 rtde_receive 包）：{e}")

    reader = UR12eReader(cfg.master)
    normalizer = DefaultNormalizer()

    # 连接（内部自动重试）
    try:
        connected = reader.connect()
        assert connected, f"主臂 RTDE 连接失败，请检查 IP：{cfg.master.ip}"
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