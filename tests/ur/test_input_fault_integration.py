# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
"""输入故障与动作生成的联调测试（P2 → 未来 P3 消费方）（P2）。

验证会话侧必须具备的三个判断：失效即撤权、恢复要新的有效快照、换代次后旧样本作废。
只用替身驱动，不连 X、不连设备。
"""

from __future__ import annotations

import pytest

from lerobot.teleoperators.ur_keyboard.input_backend import KeyboardInputBackend
from tests.ur._input_helpers import FakeDriver, ScriptedClock, make_backend, make_teleop

STEP = 0.001


def test_valid_snapshot_drives_action_then_fault_revokes():
    driver = FakeDriver(keys=("space", "1"))
    backend = make_backend(driver)
    teleop = make_teleop(joint_step_size=STEP)

    snapshot = backend.poll()
    assert snapshot.valid is True
    action = teleop.get_action_from_snapshot(snapshot)
    assert action["delta_shoulder_pan"] == pytest.approx(STEP)

    # 失焦：会话必须撤权，映射必须拒绝
    driver.focus = "no"
    faulted = backend.poll()
    assert faulted.valid is False
    with pytest.raises(ValueError):
        teleop.get_action_from_snapshot(faulted)


def test_recovery_requires_a_new_valid_snapshot():
    driver = FakeDriver(keys=("space", "1"), focus="no")
    backend = make_backend(driver)
    teleop = make_teleop(joint_step_size=STEP)
    assert backend.poll().valid is False

    driver.focus = "yes"
    recovered = backend.poll()
    assert recovered.valid is True
    action = teleop.get_action_from_snapshot(recovered)
    assert action["delta_shoulder_pan"] == pytest.approx(STEP)


def test_epoch_change_invalidates_old_snapshot():
    driver = FakeDriver(keys=("space", "2"))
    backend = make_backend(driver)
    old = backend.poll()
    assert old.valid is True

    backend.close()
    driver.window_alive = True
    backend.start()
    new = backend.poll()
    assert new.valid is True
    assert old.backend_epoch != new.backend_epoch
    # 会话侧的判据：代次不同即视为旧样本，不能沿用
    assert (old.backend_epoch, old.seq) != (new.backend_epoch, new.seq)


def test_partial_release_across_frames_keeps_constraints():
    driver = FakeDriver(keys=("space", "1", "2"))
    backend = make_backend(driver)
    teleop = make_teleop(joint_step_size=STEP)

    first = teleop.get_action_from_snapshot(backend.poll())
    assert first["delta_shoulder_pan"] == pytest.approx(STEP)
    assert first["delta_shoulder_lift"] == pytest.approx(STEP)

    driver.keys = ("space", "2")
    second = teleop.get_action_from_snapshot(backend.poll())
    assert second["delta_shoulder_pan"] == 0.0
    assert second["delta_shoulder_lift"] == pytest.approx(STEP)


def test_all_directions_released_gives_zero_action_but_space_still_held():
    driver = FakeDriver(keys=("space", "left", "up"))
    backend = make_backend(driver)
    teleop = make_teleop(cartesian_translation_step_size=STEP)

    moving = teleop.get_action_from_snapshot(backend.poll())
    assert moving["delta_x"] == pytest.approx(-STEP)
    assert moving["delta_y"] == pytest.approx(STEP)

    driver.keys = ("space",)
    stopped = teleop.get_action_from_snapshot(backend.poll())
    assert all(value == 0.0 for value in stopped.values())


def test_deadline_passed_before_step_rejects_input():
    driver = FakeDriver(keys=("space", "1"))
    clock = ScriptedClock([5_000])
    backend = make_backend(driver, clock)
    snapshot = backend.poll(deadline_ns=1_000)
    assert snapshot.reason == "deadline_exceeded"
    teleop = make_teleop()
    with pytest.raises(ValueError):
        teleop.get_action_from_snapshot(snapshot)


def test_backend_restart_after_window_close():
    driver = FakeDriver(keys=("space", "3"), window_alive=False)
    backend: KeyboardInputBackend = make_backend(driver)
    assert backend.poll().reason == "window_closed"

    backend.close()
    backend.start()
    driver.window_alive = True
    snapshot = backend.poll()
    assert snapshot.valid is True
    teleop = make_teleop(joint_step_size=STEP)
    assert teleop.get_action_from_snapshot(snapshot)["delta_elbow"] == pytest.approx(STEP)
