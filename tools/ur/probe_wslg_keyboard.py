#!/usr/bin/env python
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
"""WSLg X11 键盘探针（P2 / C3 PoC）。

这是 `input_backend.py` 的命令行外壳：键位解析、焦点判断、有效期语义全部复用正式后端，
探针自身只负责显示、人工实验流程和 XTEST 自检，不再维护第二套键位/状态实现。
不连接 UR / SO101，不启动遥操作，不导入旧运行示例。

用法：
    python tools/ur/probe_wslg_keyboard.py --preflight
    python tools/ur/probe_wslg_keyboard.py --window-seconds 120 --jsonl /tmp/c3_probe.jsonl
    python tools/ur/probe_wslg_keyboard.py --selftest

结论口径（主计划 §5）：能连 X、能建窗口都不等于键盘资格；Alt+Tab 到宿主后若拿不到
焦点失效信号，该候选不通过；每个样本有明确有效期，超过观测期限一律标 REVOKED。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections.abc import Iterable
from typing import Any

from lerobot.teleoperators.ur_keyboard.input_backend import (
    KeyboardInputBackend,
    KeyboardInputConfig,
    KeyboardInputError,
    X11KeyboardDriver,
)

PROBE_NAME = "lerobot-ur12e-c3-probe"
WINDOW_W = 760
WINDOW_H = 300


def _xtest_module() -> Any:
    try:
        from Xlib.ext import xtest
    except ImportError as exc:  # pragma: no cover - 取决于环境
        raise SystemExit("缺少 python-xlib（uv pip install python-xlib==0.33 six==1.17.0）") from exc
    return xtest


def _xtest_available(display: Any) -> bool:
    try:
        reply = display.query_extension("XTEST")
    except Exception:
        return False
    return bool(reply) and bool(getattr(reply, "present", True))


class Probe:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.config = KeyboardInputConfig(
            manage_window=not args.no_window,
            require_focus=not args.no_window,
            ttl_ms=args.ttl_ms,
            window_title=PROBE_NAME,
        )
        self.backend: KeyboardInputBackend | None = None
        self.gc: Any = None
        self.jsonl: Any = None
        self.deadline_ns = 0
        self.samples = 0
        self.invalid = 0
        self.keys_seen: set[str] = set()
        self.reasons: dict[str, int] = {}
        self.max_query_ms = 0.0
        self._last_printed = ""

    # ------------------------------------------------------------------ 生命周期
    def open(self) -> None:
        if self.args.jsonl:
            self.jsonl = open(self.args.jsonl, "w", encoding="utf-8")
        self.backend = KeyboardInputBackend(self.config)
        try:
            self.backend.start()
        except KeyboardInputError as exc:
            raise SystemExit(f"无法建立键盘后端：{exc}") from exc
        driver = self.backend.driver
        if isinstance(driver, X11KeyboardDriver) and driver.window is not None:
            screen = driver.display.screen()
            try:
                font = driver.display.open_font("fixed")
                self.gc = driver.window.create_gc(
                    foreground=screen.white_pixel, background=screen.black_pixel
                )
                self.gc.change(font=font)
            except Exception as exc:
                print(f"（窗口文字不可用：{exc}；以终端输出为准）", flush=True)
        self.deadline_ns = (
            time.monotonic_ns() + int(self.args.window_seconds * 1e9) if self.args.window_seconds else 0
        )

    def close(self) -> None:
        if self.gc is not None:
            try:
                self.gc.free()
            except Exception:
                pass
            self.gc = None
        if self.backend is not None:
            self.backend.close()
            self.backend = None
        if self.jsonl is not None:
            self.jsonl.close()
            self.jsonl = None

    # ------------------------------------------------------------------- 记录
    def record(self, payload: dict[str, Any]) -> None:
        if self.jsonl is not None:
            self.jsonl.write(json.dumps(payload, ensure_ascii=False) + "\n")
            self.jsonl.flush()

    # ------------------------------------------------------------------- 采样
    def step(self) -> tuple[Any, bool]:
        """取一份快照；到观测期限后不再产生新有效样本。"""
        now = time.monotonic_ns()
        snapshot = self.backend.poll()
        revoked = bool(self.deadline_ns and now > self.deadline_ns)
        if revoked:
            snapshot = snapshot.__class__(
                backend_epoch=snapshot.backend_epoch,
                seq=snapshot.seq,
                sample_mono_ns=now,
                valid_until_ns=now,
                pressed_keys=snapshot.pressed_keys,
                focus=snapshot.focus,
                valid=False,
                reason="observation_window_elapsed",
            )
        self.record(
            {
                "kind": "sample",
                "epoch": snapshot.backend_epoch,
                "seq": snapshot.seq,
                "mono_ns": snapshot.sample_mono_ns,
                "keys": sorted(snapshot.pressed_keys),
                "focus": snapshot.focus,
                "valid": snapshot.valid,
                "reason": snapshot.reason,
                "revoked": revoked,
                "query_ms": round(self.backend.last_query_ms, 3),
            }
        )
        self.samples += 1
        self.max_query_ms = max(self.max_query_ms, self.backend.last_query_ms)
        if not snapshot.valid:
            self.invalid += 1
            if snapshot.reason:
                self.reasons[snapshot.reason] = self.reasons.get(snapshot.reason, 0) + 1
        else:
            self.keys_seen.update(snapshot.pressed_keys)
        return snapshot, revoked

    # ------------------------------------------------------------------- 展示
    def draw(self, snapshot: Any) -> None:
        driver = self.backend.driver
        window = getattr(driver, "window", None)
        if window is None or self.gc is None:
            return
        verdict = "VALID" if snapshot.valid else f"INVALID: {snapshot.reason}"
        lines = [
            f"{PROBE_NAME}   {time.strftime('%H:%M:%S')}",
            f"keys : {', '.join(sorted(snapshot.pressed_keys)) or '-'}",
            f"focus: {snapshot.focus}",
            f"query: {self.backend.last_query_ms:.2f} ms   ttl: {self.config.ttl_ms} ms",
            f"state: {verdict}",
            f"FocusOut events: {self.backend.event_counts.get('FocusOut', 0)}",
            "实验：1 / 2 / Shift+1 / 方向键 / Space → Alt+Tab 到宿主 → 最小化 → 锁屏再回来 → 关窗",
            "focus=no 时用鼠标点一下本窗口（WSLg 的焦点由窗口管理器控制，程序抢不回来）",
        ]
        try:
            window.clear_area(0, 0, WINDOW_W, WINDOW_H)
            y = 30
            for line in lines:
                window.draw_text(self.gc, 12, y, line)
                y += 32
            driver.display.sync()
        except Exception:
            pass

    def print_line(self, snapshot: Any, revoked: bool) -> None:
        keys = ",".join(sorted(snapshot.pressed_keys)) or "-"
        verdict = "VALID" if snapshot.valid else f"INVALID({snapshot.reason})"
        tail = f"keys={keys:<26} focus={snapshot.focus:<14} {'REVOKED ' if revoked else ''}{verdict}"
        if tail != self._last_printed:
            print(f"[{time.strftime('%H:%M:%S')}] {tail}", flush=True)
            self._last_printed = tail

    def loop(self) -> int:
        frame = self.args.frame_ms / 1000.0
        while True:
            snapshot, revoked = self.step()
            self.draw(snapshot)
            self.print_line(snapshot, revoked)
            if revoked:
                print("观测期限已到，已撤权（继续观察 REVOKED 状态，Ctrl+C 结束）。", flush=True)
                self.dump_stats()
                return 0
            if getattr(self.backend.driver, "window", None) is None and not self.args.no_window:
                print("窗口已关闭（关窗请求或 DestroyNotify），探针结束。", flush=True)
                self.dump_stats()
                return 0
            time.sleep(frame)

    # ------------------------------------------------------------------- 自检
    def selftest(self) -> int:
        xtest = _xtest_module()
        driver = self.backend.driver
        display = getattr(driver, "display", None)
        if display is None:
            print("没有 X display，无法自检。", flush=True)
            return 2
        if not _xtest_available(display):
            print("XTEST 扩展不可用，无法自检。", flush=True)
            return 2
        ok = True

        def check(cond: bool, message: str) -> None:
            nonlocal ok
            print(("PASS  " if cond else "FAIL  ") + message, flush=True)
            ok = ok and cond

        focus = self.backend.poll().focus
        check(focus in ("yes", "no"), f"焦点可读（focus={focus}）")
        if isinstance(driver, X11KeyboardDriver) and driver.window is not None:
            took = driver.take_focus()
            print(
                "INFO  " + ("探针窗口取得了 X 焦点（可用焦点判断失焦）" if took else "探针窗口拿不到 X 焦点"),
                flush=True,
            )

        from Xlib import X, XK

        def keycode(symbol: str) -> int:
            return display.keysym_to_keycode(XK.string_to_keysym(symbol))

        def act(
            press_codes: Iterable[int] = (),
            release_codes: Iterable[int] = (),
            expected: set[str] | None = None,
        ) -> list[str]:
            """注入后在 0.6 s 内轮询；首个 XTEST 请求会被服务器丢弃，因此允许重发。"""
            want = expected if expected is not None else set()
            seen: list[str] = []
            for _ in range(4):
                for code in press_codes:
                    xtest.fake_input(display, X.KeyPress, code)
                for code in release_codes:
                    xtest.fake_input(display, X.KeyRelease, code)
                display.sync()
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline:
                    seen = sorted(driver.pressed_ids())
                    if set(seen) == want:
                        return seen
                    time.sleep(0.02)
            return seen

        one, shift, space, left = keycode("1"), keycode("Shift_L"), keycode("space"), keycode("Left")
        act(press_codes=[one], release_codes=[one])  # 预热

        seen = act(press_codes=[one], expected={"1"})
        check(set(seen) == {"1"}, f"按下 1：读到 ['1']（实际 {seen}）")
        seen = act(release_codes=[one], expected=set())
        check(seen == [], f"松开 1 后键集为空（实际 {seen}）")

        seen = act(press_codes=[shift, one], expected={"shift_l", "!"})
        check(set(seen) == {"shift_l", "!"}, f"Shift+1：读到 shift_l 与 '!'（实际 {seen}）")
        seen = act(release_codes=[one, shift], expected=set())
        check(seen == [], f"松开 Shift+1 后键集为空（实际 {seen}）")

        seen = act(press_codes=[left], expected={"left"})
        check(set(seen) == {"left"}, f"方向键 Left：读到 'left'（实际 {seen}）")
        act(release_codes=[left], expected=set())

        seen = act(press_codes=[space], expected={"space"})
        check(set(seen) == {"space"}, f"空格：读到 'space'（实际 {seen}）")
        act(release_codes=[space], expected=set())

        print("自检结论：" + ("全部通过（仅证明探针逻辑，不代表键盘资格）" if ok else "存在失败项"), flush=True)
        return 0 if ok else 1

    # ------------------------------------------------------------------- 统计
    def dump_stats(self) -> None:
        events = getattr(self.backend.driver, "pump_events", None) if self.backend else None
        print(
            "统计：样本 {s}，失效 {i}，最长查询 {m:.2f} ms，识别按键 {k}，失效原因 {r}".format(
                s=self.samples, i=self.invalid, m=self.max_query_ms,
                k=sorted(self.keys_seen) or "-", r=self.reasons or "-",
            ),
            flush=True,
        )
        if self.backend is not None:
            print(f"X 事件计数：{self.backend.event_counts or '-'}", flush=True)
        _ = events


def preflight() -> int:
    import os

    if not os.environ.get("DISPLAY"):
        print("FAIL  DISPLAY 未设置")
        return 1
    driver = X11KeyboardDriver(KeyboardInputConfig(manage_window=False, require_focus=False))
    try:
        driver.open()
    except KeyboardInputError as exc:
        print(f"FAIL  {exc}")
        return 1
    display = driver.display
    screen = display.screen()
    print("DISPLAY 检查：")
    for key, value in {
        "display": display.get_display_name(),
        "vendor": display.display.info.vendor,
        "screen": f"{screen.width_in_pixels}x{screen.height_in_pixels}",
        "root": hex(screen.root.id),
    }.items():
        print(f"  {key:<8} {value}")
    checks = {"XTEST": _xtest_available(display)}
    for name, call in (("XQueryKeymap", display.query_keymap), ("XGetInputFocus", display.get_input_focus)):
        try:
            call()
            checks[name] = True
        except Exception as exc:
            checks[name] = False
            print(f"  {name} 失败：{exc}")
    for name, available in checks.items():
        print(f"  {name:<13} {'OK' if available else 'MISSING'}")
    driver.close()
    ok = all(checks.values())
    print("预检结果：" + ("rc=0（只证明连得上 X，不是键盘资格）" if ok else "rc=1"))
    return 0 if ok else 1


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="WSLg X11 键盘探针（C3 PoC）")
    parser.add_argument("--preflight", action="store_true", help="只连接 X 并报告能力")
    parser.add_argument("--selftest", action="store_true", help="XTEST 合成按键自检（非资格证据）")
    parser.add_argument("--no-window", action="store_true", help="不建窗口，只在终端输出（诊断）")
    parser.add_argument("--window-seconds", type=float, default=0.0, help="整体观测期限（秒）")
    parser.add_argument("--ttl-ms", type=int, default=150, help="每个样本的有效期（毫秒）")
    parser.add_argument("--frame-ms", type=int, default=100, help="采样间隔（毫秒）")
    parser.add_argument("--jsonl", default=None, help="把样本写成 JSONL，供事后核对")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.preflight:
        return preflight()

    probe = Probe(args)
    try:
        probe.open()
        if args.selftest:
            return probe.selftest()
        if not args.window_seconds:
            print("提示：未给 --window-seconds，将一直观测直到 Ctrl+C 或关窗。", flush=True)
        return probe.loop()
    except KeyboardInterrupt:
        print("\n已中断。", flush=True)
        probe.dump_stats()
        return 0
    finally:
        probe.close()


if __name__ == "__main__":
    sys.exit(main())
