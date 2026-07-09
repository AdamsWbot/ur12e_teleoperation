# src/tests/test_mapper.py
"""mapper.py 单元测试 — 覆盖 IdentityMapper / S570Mapper / KeyboardMapper"""
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
    """快速构造 RobotState（tcp 填零）"""
    return RobotState(
        timestamp=0.0,
        joint=JointState(q=joint_rad),
        tcp_pose=Pose(0, 0, 0, 0, 0, 0),
    )


def _cmd(*joint_rad: float) -> RobotCommand:
    """快速构造 RobotCommand"""
    return RobotCommand(
        timestamp=0.0,
        joint=JointState(q=joint_rad),
        delta=JointState(q=(0.0,) * 6),
    )


# ─── IdentityMapper 测试 ────────────────────────

class TestIdentityMapper:
    def test_default_passthrough(self):
        """默认 1:1 透传"""
        m = IdentityMapper(_limits())
        result = m._map_joint(JointState(q=(1, 2, 3, 4, 5, 6)))
        assert result.q == (1, 2, 3, 4, 5, 6)

    def test_scale(self):
        """全局缩放"""
        m = IdentityMapper(_limits())
        m.set_scale(0.5)
        result = m._map_joint(JointState(q=(2, 0, 0, 0, 0, 0)))
        assert result.q[0] == 1.0

    def test_offset(self):
        """单关节偏移"""
        m = IdentityMapper(_limits())
        m.set_offset(1, 0.5)
        result = m._map_joint(JointState(q=(0, 1, 0, 0, 0, 0)))
        assert result.q[1] == 1.5

    def test_map_first_frame_delta_zero(self):
        """第一帧 delta 全零"""
        m = IdentityMapper(_limits())
        cmd = m.map(_state(1, 0, 0, 0, 0, 0), None)
        assert cmd.delta.q == (0.0,) * 6

    def test_map_delta(self):
        """正常 delta 计算"""
        m = IdentityMapper(_limits())
        prev = _cmd(0, 0, 0, 0, 0, 0)
        cmd = m.map(_state(0.1, 0, 0, 0, 0, 0), prev)
        assert abs(cmd.delta.q[0] - 0.1) < 0.001


# ─── S570Mapper 测试 ────────────────────────────

class TestS570Mapper:
    def test_default_passthrough(self):
        """默认透传（和 IdentityMapper 行为一致）"""
        m = S570Mapper(_limits())
        result = m._map_joint(JointState(q=(1, 2, 3, 4, 5, 6)))
        assert result.q == (1, 2, 3, 4, 5, 6)

    def test_direction_invert(self):
        """方向反转 j1→−j1"""
        m = S570Mapper(_limits())
        m.set_direction(0, -1)
        result = m._map_joint(JointState(q=(1, 0, 0, 0, 0, 0)))
        assert result.q[0] == -1.0

    def test_direction_multiple_invert(self):
        """多个关节同时反转"""
        m = S570Mapper(_limits())
        m.set_direction(0, -1)  # J1 reverse
        m.set_direction(2, -1)  # J3 reverse
        result = m._map_joint(JointState(q=(2, 3, 4, 0, 0, 0)))
        assert result.q[0] == -2.0
        assert result.q[1] == 3.0   # unchanged
        assert result.q[2] == -4.0

    def test_combined_transform(self):
        """direction + scale + offset 组合变换"""
        m = S570Mapper(_limits())
        m.set_direction(1, -1)   # J2 反向
        m.set_scale(0.5)          # 全局 50% 缩放
        m.set_offset(1, 0.2)      # J2 偏移 +0.2
        # J2: dir=-1 * scale=0.5 * q=1.0 + offset=0.2 = -0.5 + 0.2 = -0.3
        result = m._map_joint(JointState(q=(0, 1, 0, 0, 0, 0)))
        assert abs(result.q[1] - (-0.3)) < 0.001

    def test_direction_bad_value(self):
        """set_direction 非法值应报错"""
        m = S570Mapper(_limits())
        with pytest.raises(ValueError, match=r"\+1 或 -1"):
            m.set_direction(0, 0)

    # ── drop_joint 配置 ──────────────────────────

    def test_drop_joint_default(self):
        """默认丢弃 J7（drop_joint=6）"""
        m = S570Mapper(_limits())
        assert m.drop_joint == 6

    def test_drop_joint_custom(self):
        """设置丢弃其他关节"""
        m = S570Mapper(_limits(), drop_joint=3)
        assert m.drop_joint == 3

    def test_set_drop_joint(self):
        """运行时修改 drop_joint"""
        m = S570Mapper(_limits())
        m.set_drop_joint(0)  # 丢弃 J1
        assert m.drop_joint == 0

    def test_drop_joint_bad_value_low(self):
        """drop_joint < 0 应报错"""
        with pytest.raises(ValueError, match="0-6"):
            S570Mapper(_limits(), drop_joint=-1)

    def test_drop_joint_bad_value_high(self):
        """drop_joint > 6 应报错"""
        with pytest.raises(ValueError, match="0-6"):
            S570Mapper(_limits(), drop_joint=7)

    def test_set_drop_joint_bad_value(self):
        """运行时非法 drop_joint 应报错"""
        m = S570Mapper(_limits())
        with pytest.raises(ValueError, match="0-6"):
            m.set_drop_joint(10)


# ─── KeyboardMapper 测试 ───────────────────────

class TestKeyboardMapper:
    def test_default_passthrough(self):
        """默认透传"""
        m = KeyboardMapper(_limits())
        result = m._map_joint(JointState(q=(0.1, 0, 0, 0, 0, 0)))
        assert result.q[0] == 0.1

    def test_scale_step(self):
        """缩放步长"""
        m = KeyboardMapper(_limits())
        m.set_scale(2.0)
        result = m._map_joint(JointState(q=(0.01, 0, 0, 0, 0, 0)))
        assert result.q[0] == 0.02


# ─── 端到端：7 关节 → 标准化 → 6 关节映射 ────────

class TestEndToEnd7to6:
    """验证 S570 7 关节 → master 裁剪 → mapper 映射的完整链路"""

    def test_normalizer_drops_last_by_default(self):
        """master.py 默认丢弃最后一个关节（index 6）"""
        from src.core.master import DefaultNormalizer
        from src.common.types import RawDeviceData

        norm = DefaultNormalizer()  # drop_index=6
        raw = RawDeviceData(joint=(1, 2, 3, 4, 5, 6, 7), tcp=None)
        state = norm.normalize(raw)
        # 丢弃 index 6 = 值 7
        assert state.joint.q == (1, 2, 3, 4, 5, 6)

    def test_normalizer_drops_first(self):
        """master.py 丢弃第一个关节（index 0）"""
        from src.core.master import DefaultNormalizer
        from src.common.types import RawDeviceData

        norm = DefaultNormalizer(drop_index=0)
        raw = RawDeviceData(joint=(1, 2, 3, 4, 5, 6, 7), tcp=None)
        state = norm.normalize(raw)
        # 丢弃 index 0 = 值 1
        assert state.joint.q == (2, 3, 4, 5, 6, 7)

    def test_normalizer_drops_middle(self):
        """master.py 丢弃中间关节（index 3）"""
        from src.core.master import DefaultNormalizer
        from src.common.types import RawDeviceData

        norm = DefaultNormalizer(drop_index=3)
        raw = RawDeviceData(joint=(1, 2, 3, 4, 5, 6, 7), tcp=None)
        state = norm.normalize(raw)
        # 丢弃 index 3 = 值 4
        assert state.joint.q == (1, 2, 3, 5, 6, 7)

    def test_full_pipeline_s570_default(self):
        """完整流水线: 7关节 → normalize → S570Mapper"""
        from src.core.master import DefaultNormalizer
        from src.common.types import RawDeviceData

        # 模拟 s570.py 输出（7 关节）
        raw = RawDeviceData(joint=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7), tcp=None)

        # master.py drop_index=6 → 保留 (0.1..0.6)
        norm = DefaultNormalizer(drop_index=6)
        state = norm.normalize(raw)
        assert state.joint.q == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)

        # S570Mapper 应用方向 + 缩放
        mapper = S570Mapper(_limits())
        mapper.set_direction(0, -1)
        prev = _cmd(0, 0, 0, 0, 0, 0)
        cmd = mapper.map(state, prev)

        assert abs(cmd.joint.q[0] - (-0.1)) < 0.001  # J1 反转
        assert abs(cmd.joint.q[1] - 0.2) < 0.001     # J2 不变
        assert abs(cmd.delta.q[0] - (-0.1)) < 0.001

    def test_6_joint_input_still_works(self):
        """非 S570 设备仍传 6 关节，行为不变"""
        from src.core.master import DefaultNormalizer
        from src.common.types import RawDeviceData

        norm = DefaultNormalizer()
        raw = RawDeviceData(joint=(1, 2, 3, 4, 5, 6), tcp=None)
        state = norm.normalize(raw)
        assert state.joint.q == (1, 2, 3, 4, 5, 6)

    def test_none_joint_still_works(self):
        """joint=None 兜底行为不变"""
        from src.core.master import DefaultNormalizer
        from src.common.types import RawDeviceData

        norm = DefaultNormalizer()
        raw = RawDeviceData(joint=None, tcp=None)
        state = norm.normalize(raw)
        assert state.joint.q == (0.0,) * 6
