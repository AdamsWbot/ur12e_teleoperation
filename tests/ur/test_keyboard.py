# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
"""键盘后端与映射的正常路径测试（P2）。

不含任何 X 依赖：X11 层用 FakeDriver 替换，测试只验证后端语义与 delta_* 映射。
"""

from __future__ import annotations

import inspect
import time

import pytest

from lerobot.teleoperators.ur_keyboard import URKeyboardTeleop
from lerobot.teleoperators.ur_keyboard.input_backend import (
    InputBackendNotStartedError,
    KeyboardInputBackend,
    KeyboardInputConfig,
    KeyboardInputError,
    pressed_keys_of,
    snapshot_is_fresh,
)
from lerobot.ur_teleop_contracts import InputSnapshot

from tests.ur._input_helpers import (  # 测试私有替身，不是公共 fixture
    TTL_MS,
    FakeDriver,
    ScriptedClock,
    make_backend,
    make_teleop,
    snapshot_of,
)


# --------------------------------------------------------------------- 后端语义


def test_backend_construction_does_no_io():
    driver = FakeDriver()
    started = time.perf_counter()
    backend = KeyboardInputBackend(driver=driver)
    elapsed = time.perf_counter() - started
    assert driver.open_calls == 0
    assert backend.is_started is False
    assert backend.backend_epoch == 0
    assert elapsed < 0.2  # 构造不得连接 X


def test_poll_before_start_rejected():
    backend = KeyboardInputBackend(driver=FakeDriver())
    with pytest.raises(InputBackendNotStartedError):
        backend.poll()


def test_valid_snapshot_carries_full_state():
    driver = FakeDriver(keys=("1", "space"))
    clock = ScriptedClock([5_000_000_000, 5_000_000_000, 5_000_000_000])
    backend = make_backend(driver, clock)
    snapshot = backend.poll(deadline_ns=6_000_000_000)
    assert isinstance(snapshot, InputSnapshot)
    assert snapshot.valid is True
    assert snapshot.reason is None
    assert snapshot.pressed_keys == frozenset({"1", "space"})
    assert snapshot.focus == "yes"
    assert snapshot.backend_epoch == 1
    assert snapshot.seq == 0
    assert snapshot.sample_mono_ns == 5_000_000_000
    assert snapshot.valid_until_ns == 5_000_000_000 + TTL_MS * 1_000_000


def test_seq_monotonic_and_epoch_bumps_on_restart():
    driver = FakeDriver()
    clock = ScriptedClock()
    backend = make_backend(driver, clock)
    seqs = [backend.poll().seq for _ in range(3)]
    assert seqs == [0, 1, 2]
    backend.close()
    driver.window_alive = True
    backend.start()
    assert backend.backend_epoch == 2
    assert backend.poll().seq == 0


def test_deadline_exceeded_is_invalid_without_query():
    driver = FakeDriver(keys=("1",))
    clock = ScriptedClock([10_000_000_000])
    backend = make_backend(driver, clock)
    snapshot = backend.poll(deadline_ns=9_000_000_000)
    assert snapshot.valid is False
    assert snapshot.reason == "deadline_exceeded"
    assert snapshot.pressed_keys == frozenset()  # 还没成功读过任何键


def test_close_is_idempotent():
    driver = FakeDriver()
    backend = make_backend(driver)
    backend.close()
    backend.close()
    assert backend.is_started is False
    assert driver.close_calls >= 1


def test_close_swallows_driver_error_but_keeps_idempotent():
    driver = FakeDriver(fail=("close",))
    backend = make_backend(driver)
    backend.close()  # 不抛出
    backend.close()
    assert backend.is_started is False


# --------------------------------------------------------------------- 配置校验


def test_config_rejects_unknown_field():
    with pytest.raises(KeyboardInputError, match="未知的键盘后端配置字段"):
        KeyboardInputConfig.from_mapping({"ttl_ms": 120, "ttl_ms_typo": 1})


@pytest.mark.parametrize("ttl", [0, -1, 6000])
def test_config_rejects_bad_ttl(ttl: int):
    with pytest.raises(KeyboardInputError):
        KeyboardInputConfig(ttl_ms=ttl)


def test_config_requires_focus_only_with_window():
    with pytest.raises(KeyboardInputError):
        KeyboardInputConfig(manage_window=False, require_focus=True)
    KeyboardInputConfig(manage_window=False, require_focus=False)  # 诊断模式可以


# --------------------------------------------------------------------- 消费辅助


def test_snapshot_is_fresh_checks_valid_and_ttl():
    snapshot = InputSnapshot(
        backend_epoch=1,
        seq=0,
        sample_mono_ns=100,
        valid_until_ns=200,
        pressed_keys=frozenset({"1"}),
        focus="yes",
        valid=True,
    )
    assert snapshot_is_fresh(snapshot, 200) is True
    assert snapshot_is_fresh(snapshot, 201) is False
    invalid = InputSnapshot(1, 1, 100, 200, frozenset(), "no", False, "focus_lost")
    assert snapshot_is_fresh(invalid, 150) is False


def test_pressed_keys_of_refuses_invalid_snapshot():
    invalid = InputSnapshot(1, 0, 1, 2, frozenset({"1"}), "no", False, "focus_lost")
    with pytest.raises(KeyboardInputError, match="无效输入快照"):
        pressed_keys_of(invalid)


# --------------------------------------------------------------------- 动作映射


def test_action_features_are_per_field():
    teleop = make_teleop()
    features = teleop.action_features
    assert features["delta_shoulder_pan"] is float
    assert features["delta_x"] is float
    assert len(features) == 12
    assert "gripper" not in features
    with_gripper = make_teleop(use_gripper=True)
    assert with_gripper.action_features["gripper"] is float
    assert len(with_gripper.action_features) == 13


def test_space_plus_two_joint_keys():
    teleop = make_teleop(joint_step_size=0.001)
    action = teleop.get_action_from_snapshot(snapshot_of({"space", "1", "2"}))
    assert action["delta_shoulder_pan"] == pytest.approx(0.001)
    assert action["delta_shoulder_lift"] == pytest.approx(0.001)
    assert action["delta_elbow"] == 0.0
    assert action["delta_x"] == 0.0


def test_shift_variants_are_negative():
    teleop = make_teleop(joint_step_size=0.002)
    action = teleop.get_action_from_snapshot(snapshot_of({"!", "@"}))
    assert action["delta_shoulder_pan"] == pytest.approx(-0.002)
    assert action["delta_shoulder_lift"] == pytest.approx(-0.002)


def test_cartesian_keys_and_zero_rotation():
    teleop = make_teleop(cartesian_translation_step_size=0.003)
    action = teleop.get_action_from_snapshot(snapshot_of({"left", "up", "r", "space"}))
    assert action["delta_x"] == pytest.approx(-0.003)
    assert action["delta_y"] == pytest.approx(0.003)
    assert action["delta_z"] == pytest.approx(0.003)
    for field in ("delta_rx", "delta_ry", "delta_rz"):
        assert action[field] == 0.0


def test_partial_release_rebuilds_single_direction():
    teleop = make_teleop(joint_step_size=0.001)
    first = teleop.get_action_from_snapshot(snapshot_of({"1", "2"}))
    assert first["delta_shoulder_pan"] == pytest.approx(0.001)
    second = teleop.get_action_from_snapshot(snapshot_of({"2"}))
    assert second["delta_shoulder_pan"] == 0.0
    assert second["delta_shoulder_lift"] == pytest.approx(0.001)


def test_empty_keyset_gives_zero_action():
    teleop = make_teleop()
    action = teleop.get_action_from_snapshot(snapshot_of(set()))
    assert all(value == 0.0 for value in action.values())


def test_invalid_snapshot_rejected_by_teleop():
    teleop = make_teleop()
    with pytest.raises(ValueError, match="输入快照无效"):
        teleop.get_action_from_snapshot(snapshot_of({"1"}, valid=False, reason="focus_lost"))


def test_connect_accepts_calibrate_keyword_only():
    """connect(calibrate=...) 只为对齐官方接口，不得成为额外的校准/授权入口。"""
    signature = inspect.signature(URKeyboardTeleop.connect)
    assert "calibrate" in signature.parameters
    assert signature.parameters["calibrate"].default is True
    assert list(signature.parameters) == ["self", "calibrate"]


def test_set_input_backend_requires_protocol_methods():
    teleop = make_teleop()

    class OnlyPoll:
        def poll(self, deadline_ns=None):  # pragma: no cover - 不应被调用
            return None

    with pytest.raises(TypeError, match="KeyboardInputBackendProtocol"):
        teleop.set_input_backend(OnlyPoll())
    backend = KeyboardInputBackend(driver=FakeDriver())
    teleop.set_input_backend(backend)
    assert teleop.has_input_backend is True
