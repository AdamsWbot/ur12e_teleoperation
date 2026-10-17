# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
"""X11 键盘输入后端（P2 / C3）。

职责边界（与主计划 §3.4、task-cards §3 的 P2 段一致）：

* 只回答三件事：此刻按了什么、焦点是否在本后端窗口、这份状态的有效期到何时。
* 不维护会话状态机，不撤权，不发停止，不判断能否运动——那些是 P3 的事。
* 构造不产生任何 I/O；`start()` 才连接 X 并建立专用窗口；`close()` 幂等。
* 失效必须带原因，不能把“查询失败/失焦/超时”伪装成“操作者松开了键”。
* 焦点判断依赖 `XGetInputFocus`；本后端不注册 pynput 监听，也不做第二套键位解析。

按键 id 约定（与 `teleop_ur_keyboard.py` 的映射表一致）：
    字符键 → 小写字符本身（数字 '1'…'6'，移位后 '!'…'^'，字母 'r' / 'f' / 'q'）
    特殊键 → left / right / up / down / space / ctrl_l / ctrl_r / shift_l / shift_r /
             enter / backspace / esc / tab / alt_l / alt_r / super_l / super_r
"""

from __future__ import annotations

import logging
import os
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, fields
from typing import Any

from lerobot.ur_teleop_contracts import InputSnapshot, FocusState

logger = logging.getLogger(__name__)

# X keysym 名 → 按键 id（与探针共用同一张表，避免出现第二套键位解析）
SPECIAL_KEYSYMS: dict[str, str] = {
    "XK_Left": "left",
    "XK_Right": "right",
    "XK_Up": "up",
    "XK_Down": "down",
    "XK_space": "space",
    "XK_Control_L": "ctrl_l",
    "XK_Control_R": "ctrl_r",
    "XK_Shift_L": "shift_l",
    "XK_Shift_R": "shift_r",
    "XK_Return": "enter",
    "XK_BackSpace": "backspace",
    "XK_Escape": "esc",
    "XK_Tab": "tab",
    "XK_Alt_L": "alt_l",
    "XK_Alt_R": "alt_r",
    "XK_Super_L": "super_l",
    "XK_Super_R": "super_r",
}
MODIFIER_IDS = frozenset(
    {"shift_l", "shift_r", "ctrl_l", "ctrl_r", "alt_l", "alt_r", "super_l", "super_r"}
)
SHIFT_IDS = frozenset({"shift_l", "shift_r"})

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 200


class KeyboardInputError(RuntimeError):
    """后端自身无法建立或维持输入通路（配置/环境/依赖问题）。"""


class InputBackendNotStartedError(KeyboardInputError):
    """在 start() 之前调用 poll()。"""


def _xlib_imports() -> tuple[Any, Any, Any]:
    """按需导入 python-xlib；导入失败必须给出可操作诊断，不能静默降级。"""
    try:
        from Xlib import X, XK, display as xdisplay
    except ImportError as exc:  # pragma: no cover - 取决于环境
        raise KeyboardInputError(
            "缺少 python-xlib：uv pip install python-xlib==0.33 six==1.17.0"
        ) from exc
    return X, XK, xdisplay


def keysym_to_id(XK: Any, keysym: int) -> str | None:
    """把 X keysym 转成按键 id；无法识别的键返回 None（不猜、不兜底）。"""
    for name, key_id in SPECIAL_KEYSYMS.items():
        symbol = getattr(XK, name, None)
        if symbol is not None and keysym == symbol:
            return key_id
    text = XK.keysym_to_string(keysym)
    if not text or len(text) != 1 or not text.isprintable():
        return None
    return text.lower() if text.isalpha() else text


@dataclass(frozen=True)
class KeyboardInputConfig:
    """键盘后端参数。字段需求随 P2 提交给 P1 冻结，不在本地私改 schema。"""

    display_name: str | None = None
    manage_window: bool = True
    window_title: str = "lerobot-ur12e-input"
    ttl_ms: int = 150
    query_budget_ms: float = 30.0
    require_focus: bool = True

    def __post_init__(self) -> None:
        if not 1 <= int(self.ttl_ms) <= 5000:
            raise KeyboardInputError(f"ttl_ms 超出范围（1..5000）：{self.ttl_ms}")
        if not 0 < float(self.query_budget_ms) <= 2000:
            raise KeyboardInputError(f"query_budget_ms 超出范围（0..2000）：{self.query_budget_ms}")
        if not self.manage_window and self.require_focus:
            raise KeyboardInputError(
                "manage_window=False 时没有自己的窗口，无法要求焦点；"
                "诊断用途请同时设置 require_focus=False"
            )

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> KeyboardInputConfig:
        """只接受已知字段；未知字段明确拒绝，不静默忽略。"""
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise KeyboardInputError(f"未知的键盘后端配置字段：{unknown}")
        return cls(**dict(data))


def coerce_config(config: KeyboardInputConfig | Mapping[str, Any] | None) -> KeyboardInputConfig:
    if config is None:
        return KeyboardInputConfig()
    if isinstance(config, KeyboardInputConfig):
        return config
    if isinstance(config, Mapping):
        return KeyboardInputConfig.from_mapping(config)
    raise KeyboardInputError(f"不支持的键盘后端配置类型：{type(config)!r}")


class X11KeyboardDriver:
    """真实 X11 驱动：open() 之前不碰 X；close() 幂等。

    只暴露最少能力给后端；`display` / `window` 只为探针的可视化保留。
    """

    def __init__(self, config: KeyboardInputConfig) -> None:
        self.config = config
        self.display: Any = None
        self.window: Any = None
        self.window_id: int | None = None
        self._X: Any = None
        self._XK: Any = None
        self._opened = False
        self.last_x_error: str | None = None
        self._wm_delete_atom: int | None = None
        self._wm_protocols_atom: int | None = None

    # ------------------------------------------------------------------ 生命周期
    def open(self) -> None:
        if self._opened:
            raise KeyboardInputError("X11 驱动已打开；重复 open() 视为调用错误")
        X, XK, xdisplay = _xlib_imports()
        self._X, self._XK = X, XK
        name = self.config.display_name or os.environ.get("DISPLAY")
        if not name:
            raise KeyboardInputError("DISPLAY 未设置：当前不是 WSLg/X11 会话")
        try:
            self.display = xdisplay.Display(name)
        except Exception as exc:
            raise KeyboardInputError(f"无法连接 X display（DISPLAY={name}）：{exc}") from exc
        # X 的异步错误默认会直接打印到 stderr；这里收进 last_x_error，采样时自行判断，
        # 避免探针里有无关的协议噪音掩盖真正的失效原因。
        try:
            self.display.set_error_handler(self._record_x_error)
        except Exception:
            pass

        if self.config.manage_window:
            screen = self.display.screen()
            try:
                self.window = screen.root.create_window(
                    40,
                    40,
                    WINDOW_WIDTH,
                    WINDOW_HEIGHT,
                    0,
                    screen.root_depth,
                    X.InputOutput,
                    X.CopyFromParent,
                    background_pixel=screen.black_pixel,
                    event_mask=(
                        X.ExposureMask
                        | X.KeyPressMask
                        | X.KeyReleaseMask
                        | X.FocusChangeMask
                        | X.StructureNotifyMask
                        | X.VisibilityChangeMask
                    ),
                )
                self.window.set_wm_name(self.config.window_title)
                self.window.set_wm_class("lerobot-ur12e-input", "LerobotUr12eInput")
                try:
                    self.window.set_wm_protocols([self.display.intern_atom("WM_DELETE_WINDOW")])
                except Exception:
                    pass
                self.window.map()
                self.display.sync()
                self.window_id = int(self.window.id)
                self._wm_delete_atom = self.display.intern_atom("WM_DELETE_WINDOW")
                self.wait_viewable()
            except Exception as exc:
                self.close()
                raise KeyboardInputError(f"无法建立键盘输入窗口：{exc}") from exc
        self._opened = True
        logger.info("X11 键盘后端已打开（display=%s, window=%s）", name, hex(self.window_id or 0))

    def _record_x_error(self, error: Any, request: Any) -> None:
        self.last_x_error = f"{error}"
        logger.debug("X 协议错误：%s", error)

    def wait_viewable(self, timeout_s: float = 2.0) -> bool:
        """等窗口真正可见；对不可见窗口调用 SetInputFocus 会得到 BadMatch。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.window is None:
                return False
            try:
                if self.window.get_attributes().map_state == self._X.IsViewable:
                    return True
            except Exception:
                return False
            self.display.sync()
            time.sleep(0.02)
        return False

    def close(self) -> None:
        if self.window is not None:
            try:
                self.window.destroy()
                self.display.sync()
            except Exception:
                pass
            self.window = None
            self.window_id = None
        if self.display is not None:
            try:
                self.display.close()
            except Exception:
                pass
            self.display = None
        self._opened = False

    # ------------------------------------------------------------------- 查询
    @property
    def window_alive(self) -> bool:
        if self.window is None:
            return False
        try:
            return self.window.get_attributes().map_state == self._X.IsViewable
        except Exception:
            return False

    def pump_events(self) -> list[str]:
        """取走待处理事件，只返回事件名；关窗请求与 DestroyNotify 都会被识别。"""
        names: list[str] = []
        if self.display is None:
            return names
        while self.display.pending_events():
            event = self.display.next_event()
            if isinstance(event, int):
                names.append(f"Generic({event})")
                continue
            name = event.__class__.__name__
            if name == "ClientMessage" and self._is_wm_delete(event):
                # 用户点窗口右上角 ✕：WM 发 WM_DELETE_WINDOW，必须当作关窗处理
                names.append("WmDeleteWindow")
                self.window = None
                self.window_id = None
                continue
            names.append(name)
            if name == "DestroyNotify" and self.window is not None:
                try:
                    if getattr(event, "window", None) is not None and int(event.window.id) == self.window_id:
                        self.window = None
                except Exception:
                    pass
        return names

    def _is_wm_delete(self, event: Any) -> bool:
        try:
            if self._wm_delete_atom is None:
                return False
            client_type = getattr(event, "client_type", None)
            if self._wm_protocols_atom is None:
                self._wm_protocols_atom = self.display.intern_atom("WM_PROTOCOLS")
            if client_type != self._wm_protocols_atom:
                return False
            return int(event.data[1][0]) == int(self._wm_delete_atom)
        except Exception:
            return False

    def pressed_ids(self) -> list[str]:
        """一次 XQueryKeymap，返回完整按键集合（移位键按下时取移位层字符）。"""
        if self.display is None:
            raise KeyboardInputError("X display 未打开")
        keymap = self.display.query_keymap()
        pressed = [
            byte_index * 8 + bit
            for byte_index, byte in enumerate(keymap)
            if byte
            for bit in range(8)
            if byte & (1 << bit)
        ]
        shift_down = any(
            keysym_to_id(self._XK, self.display.keycode_to_keysym(keycode, 0)) in SHIFT_IDS
            for keycode in pressed
        )
        ids: set[str] = set()
        for keycode in pressed:
            base = keysym_to_id(self._XK, self.display.keycode_to_keysym(keycode, 0))
            shifted = keysym_to_id(self._XK, self.display.keycode_to_keysym(keycode, 1))
            if base in MODIFIER_IDS:
                ids.add(base)
            elif shift_down and shifted and shifted not in MODIFIER_IDS:
                ids.add(shifted)
            elif base:
                ids.add(base)
        return sorted(ids)

    def _ancestor_ids(self) -> set[int]:
        """本窗口及其父链（含 WM 的框架窗口）。

        WSLg 的窗口管理器会 reparent 我们的窗口，X 焦点落在框架上而不是我们直接创建的
        X window 上；只比较 window.id 会把每次采样都误判成失焦。
        """
        ids: set[int] = set()
        window = self.window
        if window is None or self.display is None:
            return ids
        try:
            root_id = int(self.display.screen().root.id)
        except Exception:
            return ids
        while window is not None:
            window_id = int(window.id)
            ids.add(window_id)
            if window_id == root_id or len(ids) > 8:
                break
            try:
                parent = window.query_tree().parent
            except Exception:
                break
            if parent is None or int(parent.id) in ids:
                break
            window = parent
        return ids

    def focus_state(self) -> FocusState:
        """yes=焦点在本后端窗口（或它的 WM 框架）上；no=已到别处；unknown=读不到。"""
        if self.display is None:
            return "unknown"
        try:
            focus = self.display.get_input_focus().focus
        except Exception:
            return "unknown"
        if isinstance(focus, int) or focus is None:
            # 0/PointerRoot：没有窗口持有键盘焦点；诊断模式（无窗口）时只能算未知
            return "no" if self.window is not None else "unknown"
        window_id = getattr(focus, "id", None)
        if not isinstance(window_id, int):
            return "unknown"
        return "yes" if window_id in self._ancestor_ids() else "no"

    def take_focus(self) -> bool:
        """尽力把 X 焦点给本窗口；有 WM 时最终由 WM 决定。"""
        if self.display is None or self.window is None:
            return False
        if not self.wait_viewable(timeout_s=1.0):
            return False
        try:
            self.display.set_input_focus(self.window, self._X.RevertToParent, self._X.CurrentTime)
            self.display.sync()
        except Exception:
            return False
        return self.focus_state() == "yes"


class KeyboardInputBackend:
    """`KeyboardInputBackendProtocol` 的 X11 实现。

    driver 可注入（测试用），因此类本身在无 X 环境里也能被构造和单测。
    """

    def __init__(
        self,
        config: KeyboardInputConfig | Mapping[str, Any] | None = None,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        driver: Any = None,
    ) -> None:
        self.config = coerce_config(config)
        self._clock = clock
        self._driver = driver
        self._started = False
        self._epoch = 0
        self._seq = 0
        self._last_keys: frozenset[str] = frozenset()
        self._last_valid_ns = 0
        self.event_counts: dict[str, int] = {}
        self.last_query_ms: float = 0.0

    # ------------------------------------------------------------------ 生命周期
    @property
    def is_started(self) -> bool:
        return self._started

    @property
    def backend_epoch(self) -> int:
        return self._epoch

    @property
    def driver(self) -> Any:
        return self._driver

    def start(self) -> None:
        """连接 X 并建立（或确认）操作窗口。重复 start 视为调用错误，避免悄悄换代次。"""
        if self._started:
            raise KeyboardInputError("键盘后端已启动；需要换代次请先 close()")
        if self._driver is None:
            self._driver = X11KeyboardDriver(self.config)
        open_method = getattr(self._driver, "open", None)
        if callable(open_method):
            open_method()
        self._started = True
        self._epoch += 1
        self._seq = 0
        self._last_keys = frozenset()
        self._last_valid_ns = 0
        self.event_counts = {}

    def close(self) -> None:
        """幂等：未启动、已关闭、部分启动都不报错。"""
        if self._driver is not None:
            try:
                self._driver.close()
            except Exception as exc:  # 关闭失败要留下痕迹，但不阻止幂等语义
                logger.warning("关闭 X11 键盘驱动时出错：%s", exc)
        self._started = False

    # ------------------------------------------------------------------- 采样
    def _snapshot(
        self,
        *,
        now_ns: int,
        seq: int,
        keys: frozenset[str],
        focus: FocusState,
        valid: bool,
        reason: str | None,
    ) -> InputSnapshot:
        return InputSnapshot(
            backend_epoch=self._epoch,
            seq=seq,
            sample_mono_ns=now_ns,
            valid_until_ns=now_ns + int(self.config.ttl_ms) * 1_000_000,
            pressed_keys=keys,
            focus=focus,
            valid=valid,
            reason=reason,
        )

    def poll(self, deadline_ns: int | None = None) -> InputSnapshot:
        """取一份完整状态。失效时给原因；调用方的 deadline 只约束“开始检查前是否已经过期”。"""
        if not self._started:
            raise InputBackendNotStartedError("键盘后端尚未 start()，不能 poll()")
        now = int(self._clock())
        seq = self._seq
        self._seq += 1

        if deadline_ns is not None and now > int(deadline_ns):
            return self._snapshot(
                now_ns=now, seq=seq, keys=self._last_keys, focus="unknown",
                valid=False, reason="deadline_exceeded",
            )

        try:
            events = self._driver.pump_events()
        except Exception as exc:
            return self._snapshot(
                now_ns=now, seq=seq, keys=self._last_keys, focus="unknown",
                valid=False, reason=f"query_failed:{type(exc).__name__}",
            )
        for name in events:
            self.event_counts[name] = self.event_counts.get(name, 0) + 1
        if "DestroyNotify" in events or "WmDeleteWindow" in events or not getattr(self._driver, "window_alive", True):
            return self._snapshot(
                now_ns=now, seq=seq, keys=self._last_keys, focus="no",
                valid=False, reason="window_closed",
            )

        started = int(self._clock())
        try:
            keys = frozenset(self._driver.pressed_ids())
        except Exception as exc:
            logger.warning("读取键态失败：%s", exc)
            return self._snapshot(
                now_ns=now, seq=seq, keys=self._last_keys, focus=self._safe_focus(),
                valid=False, reason=f"query_failed:{type(exc).__name__}",
            )
        query_ms = (int(self._clock()) - started) / 1e6
        self.last_query_ms = query_ms
        focus = self._safe_focus()

        if self.config.require_focus and focus != "yes":
            return self._snapshot(
                now_ns=now, seq=seq, keys=keys, focus=focus, valid=False, reason="focus_lost"
            )
        if query_ms > float(self.config.query_budget_ms):
            return self._snapshot(
                now_ns=now, seq=seq, keys=keys, focus=focus, valid=False, reason="late"
            )

        self._last_keys = keys
        self._last_valid_ns = now
        return self._snapshot(now_ns=now, seq=seq, keys=keys, focus=focus, valid=True, reason=None)

    def _safe_focus(self) -> FocusState:
        try:
            return self._driver.focus_state()
        except Exception:
            return "unknown"


def snapshot_is_fresh(snapshot: InputSnapshot, now_ns: int) -> bool:
    """消费方用的统一判据：既要求 valid，也要求未过期。"""
    return bool(snapshot.valid) and int(snapshot.sample_mono_ns) >= 0 and int(now_ns) <= int(
        snapshot.valid_until_ns
    )


def pressed_keys_of(snapshot: InputSnapshot) -> frozenset[str]:
    """取“当前按键”的唯一入口：失效快照直接报错，绝不退化成空集让人误读成释放。"""
    if not snapshot.valid:
        raise KeyboardInputError(
            f"无效输入快照没有可用键集（reason={snapshot.reason}）；"
            "调用方应先按 valid=False 撤权，而不是当作释放"
        )
    return snapshot.pressed_keys
