"""SafetyController.validate() 单元测试 — 覆盖所有安全检查边界情况"""

import math

import pytest

from src.common.config import ControlConfig, JointLimits
from src.common.types import ControlResult, JointState, RobotCommand
from src.core.control import SafetyController


# ─── 测试夹具 ────────────────────────────────────────────


def make_config(**overrides) -> ControlConfig:
    """构造 ControlConfig，支持字段覆盖"""
    defaults = dict(
        frequency=125,
        max_joint_velocity=1.2,
        enable_velocity_limit=True,
        enable_joint_limit=True,
        joint_limits=JointLimits(
            j1=(-6.28, 6.28),
            j2=(-6.28, 6.28),
            j3=(-3.14, 3.14),
            j4=(-6.28, 6.28),
            j5=(-6.28, 6.28),
            j6=(-6.28, 6.28),
        ),
    )
    defaults.update(overrides)
    return ControlConfig(**defaults)


def make_command(
    joint_values: tuple[float, ...],
    delta_values: tuple[float, ...] | None = None,
    timestamp: float = 1.0,
) -> RobotCommand:
    """构造 RobotCommand，delta 默认全零"""
    if delta_values is None:
        delta_values = (0.0,) * 6
    return RobotCommand(
        timestamp=timestamp,
        joint=JointState(q=tuple(joint_values)),
        delta=JointState(q=tuple(delta_values)),
    )


# ─── 测试类 ──────────────────────────────────────────────


class TestSafetyControllerValidate:
    """SafetyController.validate() 边界覆盖测试"""

    def setup_method(self):
        self.cfg = make_config()
        self.controller = SafetyController(self.cfg)

    # ── 正常路径 ──────────────────────────────────────

    def test_normal_command_passes(self):
        """正常范围内的指令应 passed=True, violations 为空"""
        cmd = make_command((0.0,) * 6)
        result = self.controller.validate(cmd)
        assert result.passed
        assert result.violations == ()

    # ── NaN / Inf 检测 ────────────────────────────────

    @pytest.mark.parametrize("bad_val", [float("nan"), float("inf"), float("-inf")])
    def test_nan_inf_triggers_emergency_stop(self, bad_val):
        """NaN/Inf 应触发 emergency_stop（全零 joint + delta）"""
        cmd = make_command((bad_val, 0.0, 0.0, 0.0, 0.0, 0.0))
        result = self.controller.validate(cmd)
        assert not result.passed
        assert "NaN/Inf" in result.violations[0]
        # emergency_stop 输出全零
        assert result.command.joint.q == (0.0,) * 6
        assert result.command.delta.q == (0.0,) * 6

    def test_nan_in_last_joint_detected(self):
        """NaN 出现在最后一个关节（j6）也应被检测到 — 验证遍历不漏检"""
        cmd = make_command((0.0, 0.0, 0.0, 0.0, 0.0, float("nan")))
        result = self.controller.validate(cmd)
        assert not result.passed
        assert "NaN/Inf" in result.violations[0]
        assert result.command.joint.q == (0.0,) * 6
        assert result.command.delta.q == (0.0,) * 6

    # ── 关节限位检查（关闭速度限幅以隔离测试）─────────

    def test_joint_below_min_is_clipped(self):
        """关节低于下限 → 截断到下限值"""
        cfg = make_config(enable_velocity_limit=False)
        ctrl = SafetyController(cfg)
        cmd = make_command((-10.0, 0.0, 0.0, 0.0, 0.0, 0.0))  # j1 min=-6.28
        result = ctrl.validate(cmd)
        assert not result.passed
        assert result.command.joint.q[0] == pytest.approx(-6.28)
        assert "Joint 1" in result.violations[0]

    def test_joint_above_max_is_clipped(self):
        """关节高于上限 → 截断到上限值"""
        cfg = make_config(enable_velocity_limit=False)
        ctrl = SafetyController(cfg)
        cmd = make_command((10.0, 0.0, 0.0, 0.0, 0.0, 0.0))  # j1 max=6.28
        result = ctrl.validate(cmd)
        assert not result.passed
        assert result.command.joint.q[0] == pytest.approx(6.28)
        assert "Joint 1" in result.violations[0]

    def test_joint3_uses_separate_limits(self):
        """j3 限位 [-3.14, 3.14] 与其他关节不同"""
        cfg = make_config(enable_velocity_limit=False)
        ctrl = SafetyController(cfg)
        cmd = make_command((0.0, 0.0, -5.0, 0.0, 0.0, 0.0))  # j3 min=-3.14
        result = ctrl.validate(cmd)
        assert not result.passed
        assert result.command.joint.q[2] == pytest.approx(-3.14)

    def test_multiple_joints_exceed_limits(self):
        """多个关节同时超限 → 全部记录在 violations 中（关速度限幅以排除干扰）"""
        cfg = make_config(enable_velocity_limit=False)
        ctrl = SafetyController(cfg)
        cmd = make_command((-10.0, 10.0, -5.0, 0.0, 0.0, 0.0))
        result = ctrl.validate(cmd)
        assert not result.passed
        assert len(result.violations) == 3

    def test_joint_clipping_updates_delta(self):
        """关节截断后 delta 应同步更新（关速度限幅以隔离测试）"""
        cfg = make_config(enable_velocity_limit=False)
        ctrl = SafetyController(cfg)

        # prev_q = 10.0 - 0.0 = 10.0，j1 超上限 → 截断到 6.28，delta = 6.28 - 10.0 = -3.72
        cmd = make_command((10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        result = ctrl.validate(cmd)
        assert not result.passed
        assert result.command.delta.q[0] == pytest.approx(6.28 - 10.0)

    def test_joint_at_limit_boundary_passes(self):
        """关节位置恰好等于限位边界值 → 通过，不触发 violation（关速度限幅以隔离）"""
        cfg = make_config(enable_velocity_limit=False)
        ctrl = SafetyController(cfg)
        # j1 下限 -6.28，上限 6.28
        cmd = make_command((-6.28, 6.28, 0.0, 0.0, 0.0, 0.0))
        result = ctrl.validate(cmd)
        assert result.passed
        assert result.violations == ()
        # 值保持不变
        assert result.command.joint.q[0] == pytest.approx(-6.28)
        assert result.command.joint.q[1] == pytest.approx(6.28)

    # ── 速度限幅检查 ──────────────────────────────────

    def test_velocity_exceeded_is_scaled(self):
        """速度超限 → delta 按比例缩减，joint 同步更新"""
        dt = 1.0 / 125  # 0.008
        # delta=0.5 → vel = 0.5/0.008 = 62.5 >> max_vel=1.2
        # scale = 1.2/62.5 = 0.0192
        cmd = make_command(
            joint_values=(0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
            delta_values=(0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        result = self.controller.validate(cmd)

        assert not result.passed
        expected_scale = 1.2 / (0.5 / dt)
        expected_delta = 0.5 * expected_scale
        assert result.command.delta.q[0] == pytest.approx(expected_delta)
        # joint 应同步更新：new_joint = prev_q + new_delta = 0.0 + expected_delta
        assert result.command.joint.q[0] == pytest.approx(expected_delta)

    def test_velocity_near_limit_passes(self):
        """速度接近但未超限 → 通过"""
        dt = 1.0 / 125
        max_vel = 1.2
        # delta = max_vel * dt * 0.99 = 刚好在限速内
        safe_delta = max_vel * dt * 0.99
        cmd = make_command(
            joint_values=(safe_delta, 0.0, 0.0, 0.0, 0.0, 0.0),
            delta_values=(safe_delta, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        result = self.controller.validate(cmd)
        assert result.passed

    def test_multiple_joints_exceed_velocity(self):
        """多关节同时超速 → 各关节独立缩减"""
        cmd = make_command(
            joint_values=(0.5, -0.3, 0.0, 0.0, 0.0, 0.0),
            delta_values=(0.5, -0.3, 0.0, 0.0, 0.0, 0.0),
        )
        result = self.controller.validate(cmd)
        assert not result.passed
        assert len(result.violations) == 2
        # 两个关节的 delta 都应被缩减
        assert abs(result.command.delta.q[0]) < 0.5
        assert abs(result.command.delta.q[1]) < 0.3

    def test_velocity_scale_preserves_direction(self):
        """速度缩减后方向不变（正负号保留）"""
        dt = 1.0 / 125
        cmd = make_command(
            joint_values=(-0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
            delta_values=(-0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        result = self.controller.validate(cmd)
        assert result.command.delta.q[0] < 0  # 保持负方向
        assert result.command.joint.q[0] < 0  # joint 也保持负方向

    def test_joint_and_velocity_violations_together(self):
        """同一关节同时超限和超速 → 两种 violation 都被记录，最终值以限位截断为准"""
        # q[0]=10.0 > j1_max=6.28 → 超限；delta[0]=2.0 → 超速 (vel=2.0/0.008=250 >> 1.2)
        # 预期：先限位截断到 6.28，delta 同步为 6.28-prev_q，再对 delta 做速度缩放
        cmd = make_command(
            joint_values=(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            delta_values=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        result = self.controller.validate(cmd)
        assert not result.passed
        # 应有两条 violation：限位 + 超速
        violation_msgs = " ".join(result.violations).lower()
        assert "exceed" in violation_msgs  # 限位违规
        assert "velocity" in violation_msgs or "vel" in violation_msgs  # 超速违规
        assert len(result.violations) >= 2

    # ── 开关关闭 ──────────────────────────────────────

    def test_joint_limit_disabled(self):
        """enable_joint_limit=False → 不截断"""
        cfg = make_config(enable_joint_limit=False)
        controller = SafetyController(cfg)
        cmd = make_command((-10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        result = controller.validate(cmd)
        assert result.passed
        assert result.command.joint.q[0] == pytest.approx(-10.0)  # 未截断

    def test_velocity_limit_disabled(self):
        """enable_velocity_limit=False → 不缩减"""
        cfg = make_config(enable_velocity_limit=False)
        controller = SafetyController(cfg)
        cmd = make_command(
            joint_values=(0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
            delta_values=(0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        result = controller.validate(cmd)
        assert result.passed  # 速度未检查
        assert result.command.delta.q[0] == pytest.approx(0.5)  # 未缩减

    def test_zero_frequency_fallback_dt(self):
        """frequency=0 时 _dt 应回退为 0.01，不抛出异常"""
        cfg = make_config(frequency=0)
        controller = SafetyController(cfg)
        # _dt 应为 0.01（回退值），而非除零错误
        assert controller._dt == pytest.approx(0.01)
        # 验证速度限位仍能正常工作
        cmd = make_command(
            joint_values=(0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
            delta_values=(0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        # dt=0.01 → vel=0.1/0.01=10 > 1.2 → 应缩减
        result = controller.validate(cmd)
        assert not result.passed
        assert result.command.delta.q[0] < 0.1

    # ── joint 与 delta 一致性验证 ─────────────────────

    def test_joint_delta_consistency_after_all_checks(self):
        """最终输出的 joint 应等于 prev_q + delta（一致性不变量）"""
        # 构造一个会触发限位+限速的极端指令
        cmd = make_command(
            joint_values=(10.0, -10.0, 0.0, 0.0, 0.0, 0.0),
            delta_values=(2.0, -2.0, 0.0, 0.0, 0.0, 0.0),
        )
        result = self.controller.validate(cmd)
        # 验证不变量：对于每个关节，joint ≈ prev_q + delta
        for i in range(6):
            prev_q = cmd.joint.q[i] - cmd.delta.q[i]
            assert result.command.joint.q[i] == pytest.approx(
                prev_q + result.command.delta.q[i]
            )


class TestSafetyControllerEmergencyStop:
    """emergency_stop() 基础功能测试"""

    def test_emergency_stop_returns_all_zero(self):
        controller = SafetyController(make_config())
        cmd = controller.emergency_stop()
        assert cmd.joint.q == (0.0,) * 6
        assert cmd.delta.q == (0.0,) * 6
        assert cmd.timestamp == 0.0

    def test_emergency_stop_with_custom_timestamp(self):
        """emergency_stop(timestamp=42.0) 应保留自定义时间戳"""
        controller = SafetyController(make_config())
        cmd = controller.emergency_stop(timestamp=42.0)
        assert cmd.timestamp == pytest.approx(42.0)
        assert cmd.joint.q == (0.0,) * 6
        assert cmd.delta.q == (0.0,) * 6
