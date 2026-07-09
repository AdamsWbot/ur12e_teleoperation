# src/tests/test_mapper.py
"""
mapper.py 单元测试

═══════════════════════════════════════════════════════════════
  数据流（S570 遥操作全链路 — 计划2）
═══════════════════════════════════════════════════════════════

  config.yaml
    │  s570.joint_mapping=[1,2,3,4,5,6]    选择哪6个S570关节
    │  s570.joint_direction=[1,1,1,1,1,1]  关节符号校准
    │  s570.enable_calibration=true         启动时自动零点校准
    │  mapper.scale=1.0                     主从运动范围比
    │  mapper.joint_offset=[0,0,0,0,0,0]   穿戴零位偏差
    ▼
  s570.py
    │  S570Reader(cfg)
    │  read(): 串口读7关节 → joint_mapping选6个 → 度→弧度
    │  RawDeviceData(joint=6轴弧度, tcp=None, buttons=位掩码)
    ▼
  master.py
    │  DefaultNormalizer.normalize()
    │  时间戳(time.monotonic) + 缺字段补零
    │  RobotState(timestamp, joint=6轴, tcp_pose)
    ▼
  mapper.py — 本文件测试对象
    │  run_system.py 启动时:
    │    if s570.enable_calibration:
    │        mapper.calibrate(state.joint)     ← 记录零位偏移
    │  每帧循环:
    │    cmd = mapper.map(state, prev_cmd)
    │      _apply_transform: q_out[i] = dir[i] × scale × q_in[i] + offset[i]
    │      _compute_delta:    delta[i] = current[i] - previous[i]
    │    RobotCommand(timestamp, joint=6轴, delta=6轴)
    ▼
  control.py → filter.py → slave.py → UR12e 从臂 (servoJ, 125Hz)

═══════════════════════════════════════════════════════════════
"""
import math
import pytest

from src.common.types import JointState, RobotState, RobotCommand, Pose
from src.common.config import JointLimits
from src.core.mapper import IdentityMapper, S570Mapper, KeyboardMapper


# ─── 共享夹具 ───────────────────────────────────

def _limits():
    return JointLimits(
        j1=(-6.28, 6.28), j2=(-6.28, 6.28), j3=(-3.14, 3.14),
        j4=(-6.28, 6.28), j5=(-6.28, 6.28), j6=(-6.28, 6.28),
    )


def _state(*joint_rad: float) -> RobotState:
    return RobotState(
        timestamp=0.0,
        joint=JointState(q=joint_rad),
        tcp_pose=Pose(0, 0, 0, 0, 0, 0),
    )


def _cmd(*joint_rad: float) -> RobotCommand:
    return RobotCommand(
        timestamp=0.0,
        joint=JointState(q=joint_rad),
        delta=JointState(q=(0.0,) * 6),
    )


# ═══════════════════════════════════════════════════════
#  IdentityMapper
# ═══════════════════════════════════════════════════════

class TestIdentityMapper:
    """UR12e 同构 1:1 映射"""

    def test_default_passthrough(self):
        m = IdentityMapper(_limits())
        assert m._map_joint(JointState(q=(1, 2, 3, 4, 5, 6))).q == (1, 2, 3, 4, 5, 6)

    def test_scale(self):
        m = IdentityMapper(_limits())
        m.set_scale(0.5)
        assert m._map_joint(JointState(q=(2, 0, 0, 0, 0, 0))).q[0] == 1.0

    def test_offset(self):
        m = IdentityMapper(_limits())
        m.set_offset(1, 0.5)
        assert m._map_joint(JointState(q=(0, 1, 0, 0, 0, 0))).q[1] == 1.5

    def test_first_frame_delta_zero(self):
        cmd = IdentityMapper(_limits()).map(_state(1, 0, 0, 0, 0, 0), None)
        assert cmd.delta.q == (0.0,) * 6

    def test_delta(self):
        m = IdentityMapper(_limits())
        cmd = m.map(_state(0.1, 0, 0, 0, 0, 0), _cmd(0, 0, 0, 0, 0, 0))
        assert abs(cmd.delta.q[0] - 0.1) < 0.001


# ═══════════════════════════════════════════════════════
#  calibrate（所有 Mapper 共用 — 计划2 零点校准）
# ═══════════════════════════════════════════════════════

class TestCalibrate:
    """计划2: run_system.py 启动时调用 mapper.calibrate(state.joint)"""

    def test_calibrate_makes_current_pose_zero(self):
        """校准后当前姿态映射为全零指令"""
        m = IdentityMapper(_limits())
        # 穿戴后的初始姿态
        init_joint = JointState(q=(0.5, -0.3, 1.2, 0.0, -0.8, 0.6))
        m.calibrate(init_joint)
        # 校准后，同样的关节输入 → 输出全零
        result = m._apply_transform(init_joint)
        for i in range(6):
            assert abs(result.q[i]) < 1e-9, f"joint[{i}]={result.q[i]} 应接近 0"

    def test_calibrate_with_direction(self):
        """校准正确考虑方向反转"""
        m = S570Mapper(_limits())
        m.set_direction(0, -1)
        init = JointState(q=(0.5, 0, 0, 0, 0, 0))
        m.calibrate(init)
        # offset[0] = -(-1) * 1.0 * 0.5 = 0.5
        # transform: dir=-1 * scale=1 * 0.5 + offset=0.5 = 0
        result = m._apply_transform(init)
        assert abs(result.q[0]) < 1e-9

    def test_calibrate_then_move(self):
        """校准后，相对运动正确传递"""
        m = IdentityMapper(_limits())
        init = JointState(q=(0.5, 0, 0, 0, 0, 0))
        m.calibrate(init)
        # 从校准位置移动 +0.1 rad
        moved = JointState(q=(0.6, 0, 0, 0, 0, 0))
        result = m._apply_transform(moved)
        assert abs(result.q[0] - 0.1) < 1e-9
        # 其他关节不变
        for i in range(1, 6):
            assert abs(result.q[i]) < 1e-9


# ═══════════════════════════════════════════════════════
#  S570Mapper
# ═══════════════════════════════════════════════════════

class TestS570Mapper:
    """S570 外骨骼 → UR12e 映射变换"""

    def test_default_passthrough(self):
        m = S570Mapper(_limits())
        assert m._map_joint(JointState(q=(1, 2, 3, 4, 5, 6))).q == (1, 2, 3, 4, 5, 6)

    def test_direction_invert(self):
        m = S570Mapper(_limits())
        m.set_direction(0, -1)
        assert m._map_joint(JointState(q=(1, 0, 0, 0, 0, 0))).q[0] == -1.0

    def test_direction_multiple(self):
        m = S570Mapper(_limits())
        m.set_direction(0, -1)
        m.set_direction(2, -1)
        result = m._map_joint(JointState(q=(2, 3, 4, 0, 0, 0)))
        assert result.q[0] == -2.0
        assert result.q[1] == 3.0
        assert result.q[2] == -4.0

    def test_combined_transform(self):
        """dir=-1 × scale=0.5 × q=1.0 + offset=0.2 = -0.3"""
        m = S570Mapper(_limits())
        m.set_direction(1, -1)
        m.set_scale(0.5)
        m.set_offset(1, 0.2)
        result = m._map_joint(JointState(q=(0, 1, 0, 0, 0, 0)))
        assert abs(result.q[1] - (-0.3)) < 0.001

    def test_direction_bad_value(self):
        m = S570Mapper(_limits())
        with pytest.raises(ValueError, match=r"\+1 或 -1"):
            m.set_direction(0, 0)

    # ── set_all_directions（来自 config.yaml）─────

    def test_set_all_directions(self):
        m = S570Mapper(_limits())
        m.set_all_directions((1, -1, 1, -1, 1, -1))
        result = m._map_joint(JointState(q=(1, 2, 3, 4, 5, 6)))
        assert result.q == (1, -2, 3, -4, 5, -6)

    def test_set_all_directions_bad_length(self):
        m = S570Mapper(_limits())
        with pytest.raises(ValueError, match="恰好 6"):
            m.set_all_directions((1, 1, 1, 1, 1))

    def test_set_all_directions_bad_value(self):
        m = S570Mapper(_limits())
        with pytest.raises(ValueError, match="±1"):
            m.set_all_directions((1, 1, 1, 1, 1, 0))

    # ── set_joint_mapping（来自 config.yaml）─────

    def test_set_joint_mapping_default(self):
        m = S570Mapper(_limits())
        assert m.joint_mapping == (1, 2, 3, 4, 5, 6)

    def test_set_joint_mapping_custom(self):
        m = S570Mapper(_limits())
        m.set_joint_mapping((2, 3, 4, 5, 6, 7))
        assert m.joint_mapping == (2, 3, 4, 5, 6, 7)

    def test_set_joint_mapping_bad_length(self):
        m = S570Mapper(_limits())
        with pytest.raises(ValueError, match="恰好 6"):
            m.set_joint_mapping((1, 2, 3, 4, 5))

    def test_set_joint_mapping_out_of_range(self):
        m = S570Mapper(_limits())
        with pytest.raises(ValueError, match="1-7"):
            m.set_joint_mapping((1, 2, 3, 4, 5, 8))

    def test_set_joint_mapping_duplicate(self):
        m = S570Mapper(_limits())
        with pytest.raises(ValueError, match="重复"):
            m.set_joint_mapping((1, 1, 3, 4, 5, 6))

    def test_joint_mapping_link_to_config(self):
        """验证 mapper.joint_mapping 与 cfg.s570.joint_mapping 使用同一数据"""
        from src.common.config import load_app_config
        cfg = load_app_config("config/config.yaml")
        m = S570Mapper(_limits())
        m.set_joint_mapping(cfg.s570.joint_mapping)
        assert m.joint_mapping == tuple(cfg.s570.joint_mapping)


# ═══════════════════════════════════════════════════════
#  KeyboardMapper
# ═══════════════════════════════════════════════════════

class TestKeyboardMapper:
    def test_default_passthrough(self):
        assert KeyboardMapper(_limits())._map_joint(
            JointState(q=(0.1, 0, 0, 0, 0, 0))).q[0] == 0.1

    def test_scale_step(self):
        m = KeyboardMapper(_limits())
        m.set_scale(2.0)
        assert m._map_joint(JointState(q=(0.01, 0, 0, 0, 0, 0))).q[0] == 0.02


# ═══════════════════════════════════════════════════════
#  S570 joint_mapping 7→6 裁剪（纯逻辑，模拟 s570.py）
# ═══════════════════════════════════════════════════════

class TestJointMapping:
    """验证 s570.py 的 joint_mapping 逻辑 — 与 config.yaml 对应"""

    # 模拟 S570 读取的 7 个关节（度）
    _ANGLES_7 = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]

    def _select(self, mapping):
        return [self._ANGLES_7[i - 1] for i in mapping]

    def test_default_keep_first_6(self):
        """joint_mapping=[1,2,3,4,5,6] → 丢弃 J7"""
        assert self._select((1, 2, 3, 4, 5, 6)) == [10, 20, 30, 40, 50, 60]

    def test_keep_last_6(self):
        """joint_mapping=[2,3,4,5,6,7] → 丢弃 J1"""
        assert self._select((2, 3, 4, 5, 6, 7)) == [20, 30, 40, 50, 60, 70]

    def test_reorder(self):
        """j2↔j3 交换顺序"""
        assert self._select((1, 3, 2, 4, 5, 6)) == [10, 30, 20, 40, 50, 60]

    def test_full_pipeline(self):
        """s570 joint_mapping → mapper 完整流水线"""
        # s570.py: joint_mapping 选择 6 关节，度→弧度
        mapping = (1, 2, 3, 4, 5, 6)
        joint_rad = tuple(math.radians(self._ANGLES_7[i - 1]) for i in mapping)

        # master.py: 标准化 → RobotState
        state = _state(*joint_rad)

        # mapper.py: calibrate(当前姿态) → 零位
        mapper = S570Mapper(_limits())
        mapper.calibrate(state.joint)

        # 校准后，同一姿态输出为零
        cmd = mapper.map(state, None)
        for i in range(6):
            assert abs(cmd.joint.q[i]) < 1e-9

        # 关节移动后，输出相对偏移
        moved_deg = [15.0, 25.0, 35.0, 45.0, 55.0, 65.0]
        moved_rad = tuple(math.radians(d) for d in moved_deg)
        state2 = _state(*moved_rad)
        cmd2 = mapper.map(state2, cmd)
        # delta = 5° = 0.08727 rad
        for i in range(6):
            assert abs(cmd2.delta.q[i] - math.radians(5.0)) < 0.01
