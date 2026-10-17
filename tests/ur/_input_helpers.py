# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
"""P2 键盘测试的私有替身与工厂（不属于公共 fixture，不放 conftest）。

只被 tests/ur/test_keyboard*.py 与 test_input_fault_integration.py 使用。
"""

from __future__ import annotations

from lerobot.teleoperators.ur_keyboard import URKeyboardTeleop, URKeyboardTeleopConfig
from lerobot.teleoperators.ur_keyboard.input_backend import KeyboardInputBackend, KeyboardInputConfig
from lerobot.ur_teleop_contracts import InputSnapshot

TTL_MS = 150


class ScriptedClock:
    """可脚本化的时钟：按顺序返回值，耗尽后固定为最后一个值。"""

    def __init__(self, values: list[int] | None = None, default: int = 1_000_000_000) -> None:
        self.values = list(values or [])
        self.default = default

    def __call__(self) -> int:
        if self.values:
            return self.values.pop(0)
        return self.default


class FakeDriver:
    """替身 X11 驱动：模拟键态、焦点、事件与故障。"""

    def __init__(
        self,
        *,
        keys: tuple[str, ...] = (),
        focus: str = "yes",
        events: list[str] | None = None,
        window_alive: bool = True,
        fail: tuple[str, ...] = (),
    ) -> None:
        self.keys = keys
        self.focus = focus
        self.events = list(events or [])
        self.window_alive = window_alive
        self.fail = set(fail)
        self.open_calls = 0
        self.close_calls = 0
        self.closed = False

    def open(self) -> None:
        self.open_calls += 1
        self.closed = False

    def close(self) -> None:
        self.close_calls += 1
        self.closed = True
        if "close" in self.fail:
            raise RuntimeError("close 失败（模拟）")

    def pump_events(self) -> list[str]:
        if "pump" in self.fail:
            raise RuntimeError("pump 失败（模拟）")
        return list(self.events)

    def pressed_ids(self) -> list[str]:
        if "pressed" in self.fail:
            raise OSError("query_keymap 失败（模拟）")
        return list(self.keys)

    def focus_state(self) -> str:
        if "focus" in self.fail:
            raise RuntimeError("focus 失败（模拟）")
        return self.focus


def make_backend(
    driver: FakeDriver, clock: ScriptedClock | None = None, ttl_ms: int = TTL_MS
) -> KeyboardInputBackend:
    backend = KeyboardInputBackend(
        KeyboardInputConfig(ttl_ms=ttl_ms), clock=clock or ScriptedClock(), driver=driver
    )
    backend.start()
    return backend


def make_teleop(**kwargs) -> URKeyboardTeleop:
    return URKeyboardTeleop(URKeyboardTeleopConfig(**kwargs))


def snapshot_of(
    keys: set[str],
    *,
    valid: bool = True,
    reason: str | None = None,
    epoch: int = 3,
    seq: int = 7,
    sample_mono_ns: int = 1,
    valid_until_ns: int = 2,
) -> InputSnapshot:
    return InputSnapshot(
        backend_epoch=epoch,
        seq=seq,
        sample_mono_ns=sample_mono_ns,
        valid_until_ns=valid_until_ns,
        pressed_keys=frozenset(keys),
        focus="yes" if valid else "no",
        valid=valid,
        reason=reason,
    )
