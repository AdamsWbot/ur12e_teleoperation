# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
"""输入故障测试：丢 release、读取冻结、过期、乱序/重放、关窗、查询异常（P2）。

全部用替身驱动，不建窗口、不连设备。
"""

from __future__ import annotations

import pytest

from lerobot.teleoperators.ur_keyboard import URKeyboardTeleop
from lerobot.teleoperators.ur_keyboard.input_backend import (
    KeyboardInputError,
    pressed_keys_of,
    snapshot_is_fresh,
)
from tests.ur._input_helpers import TTL_MS, FakeDriver, ScriptedClock, make_backend, make_teleop


def _is_newer(candidate, current) -> bool:
    """消费方判断“这份快照是否比手上的更新”：同代次且序号更大。"""
    return candidate.backend_epoch == current.backend_epoch and candidate.seq > current.seq


def test_dropped_release_does_not_become_a_release():
    """丢了 release（物理键还按着，但焦点已丢）：必须失效，且旧键集不能被当成“松开”。"""
    driver = FakeDriver(keys=("1",), focus="no")
    backend = make_backend(driver)
    snapshot = backend.poll()
    assert snapshot.valid is False
    assert snapshot.reason == "focus_lost"
    assert snapshot.pressed_keys == frozenset({"1"})  # 仅作诊断保留
    with pytest.raises(KeyboardInputError):
        pressed_keys_of(snapshot)
    teleop: URKeyboardTeleop = make_teleop()
    with pytest.raises(ValueError, match="输入快照无效"):
        teleop.get_action_from_snapshot(snapshot)


def test_frozen_query_keeps_failing_with_reason_and_monotonic_seq():
    driver = FakeDriver(fail=("pressed",))
    clock = ScriptedClock()
    backend = make_backend(driver, clock)
    seqs, reasons = [], set()
    for _ in range(5):
        snapshot = backend.poll()
        seqs.append(snapshot.seq)
        reasons.add(snapshot.reason)
        assert snapshot.valid is False
    assert seqs == [0, 1, 2, 3, 4]
    assert reasons == {"query_failed:OSError"}
    assert backend.backend_epoch == 1  # 冻结不换 epoch


def test_query_failure_after_success_keeps_last_keys_but_stays_invalid():
    driver = FakeDriver(keys=("1", "space"))
    backend = make_backend(driver)
    good = backend.poll()
    assert good.valid and good.pressed_keys == frozenset({"1", "space"})

    driver.fail.add("pressed")
    bad = backend.poll()
    assert bad.valid is False
    assert bad.reason == "query_failed:OSError"
    assert bad.pressed_keys == frozenset({"1", "space"})  # 最后一次成功读取，仅诊断
    with pytest.raises(KeyboardInputError):
        pressed_keys_of(bad)


def test_snapshot_expires_after_ttl():
    clock = ScriptedClock([1_000_000_000, 1_000_000_000, 1_000_000_000])
    backend = make_backend(FakeDriver(keys=("2",)), clock)
    snapshot = backend.poll()
    assert snapshot_is_fresh(snapshot, snapshot.valid_until_ns) is True
    assert snapshot_is_fresh(snapshot, snapshot.valid_until_ns + 1) is False
    assert snapshot.valid_until_ns - snapshot.sample_mono_ns == TTL_MS * 1_000_000


def test_late_query_is_invalid():
    late_after = 200 * 1_000_000  # 200 ms > query_budget_ms(30) 与 ttl_ms(150)
    clock = ScriptedClock([0, 0, late_after])
    backend = make_backend(FakeDriver(keys=("1",)), clock)
    snapshot = backend.poll()
    assert snapshot.valid is False
    assert snapshot.reason == "late"


def test_out_of_order_and_replay_detected_by_seq():
    clock = ScriptedClock()
    backend = make_backend(FakeDriver(keys=("1",)), clock)
    first = backend.poll()
    second = backend.poll()
    assert _is_newer(second, first) is True
    assert _is_newer(first, second) is False  # 重放旧样本必须被识别
    replayed = first.__class__(
        backend_epoch=first.backend_epoch,
        seq=first.seq,
        sample_mono_ns=first.sample_mono_ns,
        valid_until_ns=first.valid_until_ns,
        pressed_keys=first.pressed_keys,
        focus=first.focus,
        valid=first.valid,
        reason=first.reason,
    )
    assert _is_newer(replayed, second) is False


def test_window_closed_is_sticky_until_restart():
    driver = FakeDriver(window_alive=False)
    backend = make_backend(driver)
    for _ in range(3):
        snapshot = backend.poll()
        assert snapshot.valid is False
        assert snapshot.reason == "window_closed"
        assert snapshot.focus == "no"
    backend.close()  # 换代次必须显式 close() → start()，不允许悄悄重启
    backend.start()
    driver.window_alive = True
    assert backend.poll().valid is True


def test_destroy_notify_marks_window_closed():
    driver = FakeDriver(events=["DestroyNotify"])
    backend = make_backend(driver)
    snapshot = backend.poll()
    assert snapshot.reason == "window_closed"


def test_wm_delete_window_marks_window_closed():
    """用户点窗口右上角 ✕ 时 WM 发 WM_DELETE_WINDOW，必须立刻判为关窗。"""
    driver = FakeDriver(events=["WmDeleteWindow"])
    backend = make_backend(driver)
    snapshot = backend.poll()
    assert snapshot.valid is False
    assert snapshot.reason == "window_closed"
    assert snapshot.focus == "no"


def test_pump_failure_reports_query_failed():
    backend = make_backend(FakeDriver(fail=("pump",)))
    snapshot = backend.poll()
    assert snapshot.valid is False
    assert snapshot.reason == "query_failed:RuntimeError"


def test_focus_state_failure_degrades_to_unknown_and_revokes():
    driver = FakeDriver(keys=("1",), fail=("focus",))
    backend = make_backend(driver)
    snapshot = backend.poll()
    assert snapshot.valid is False
    assert snapshot.focus == "unknown"
    assert snapshot.reason == "focus_lost"


def test_pointerroot_or_no_focus_is_not_valid():
    driver = FakeDriver(keys=("space",), focus="no")
    backend = make_backend(driver)
    assert backend.poll().valid is False
    driver.focus = "yes"
    assert backend.poll().valid is True


def test_start_with_key_already_held_is_visible_and_not_authorised():
    """T20：启动前就按住 Space，第一份快照必须如实带出这个键（不能当成没按）。"""
    backend = make_backend(FakeDriver(keys=("space", "1")))
    snapshot = backend.poll()
    assert snapshot.valid is True
    assert snapshot.pressed_keys == frozenset({"space", "1"})


def test_held_key_while_focus_lost_is_not_faked_as_release():
    """T18：键仍按着时失焦，不能把旧键集当成“操作者松开了”。"""
    backend = make_backend(FakeDriver(keys=("1",), focus="no"))
    snapshot = backend.poll()
    assert snapshot.valid is False
    assert snapshot.reason == "focus_lost"
    assert snapshot.pressed_keys == frozenset({"1"})
    with pytest.raises(KeyboardInputError):
        pressed_keys_of(snapshot)


def test_deadline_exceeded_does_not_query_driver():
    driver = FakeDriver(keys=("1",))
    backend = make_backend(driver, ScriptedClock([50]))
    driver.fail.add("pressed")  # 若仍去查询就会抛错，从而暴露“不该查询”
    snapshot = backend.poll(deadline_ns=10)
    assert snapshot.reason == "deadline_exceeded"
