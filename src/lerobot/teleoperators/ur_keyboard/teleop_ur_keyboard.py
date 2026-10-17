#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.

import logging
import os
import sys
import time
from queue import Queue
from typing import Any

from lerobot.ur_teleop_contracts import InputSnapshot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from ..utils import TeleopEvents
from .configuration_ur_keyboard import URKeyboardTeleopConfig

# 监听按需加载：模块导入时不碰 pynput（它会连 X），只有真正要开监听时才导入。
_PYNPUT: dict[str, Any] = {"loaded": False, "keyboard": None, "error": None}

# 按键 id 约定（与 input_backend.py 的 X11 键位解析一致）：
#   字符键 → 小写字符本身（'1'…'6'、移位后 '!'…'^'、'r' / 'f' / 'q'）
#   特殊键 → left / right / up / down / space / ctrl_l / ctrl_r / enter / backspace / esc
JOINT_KEYS = {
    "1": ("delta_shoulder_pan", 1),
    "2": ("delta_shoulder_lift", 1),
    "3": ("delta_elbow", 1),
    "4": ("delta_wrist_1", 1),
    "5": ("delta_wrist_2", 1),
    "6": ("delta_wrist_3", 1),
    "!": ("delta_shoulder_pan", -1),
    "@": ("delta_shoulder_lift", -1),
    "#": ("delta_elbow", -1),
    "$": ("delta_wrist_1", -1),
    "%": ("delta_wrist_2", -1),
    "^": ("delta_wrist_3", -1),
}
CARTESIAN_KEYS = {
    "left": ("delta_x", -1),
    "right": ("delta_x", 1),
    "up": ("delta_y", 1),
    "down": ("delta_y", -1),
    "r": ("delta_z", 1),
    "f": ("delta_z", -1),
}
ACTION_KEY_IDS = tuple(JOINT_KEYS) + tuple(CARTESIAN_KEYS) + ("space", "ctrl_l", "ctrl_r")


def load_pynput() -> Any:
    """按需导入 pynput；失败原因留在 _PYNPUT['error'] 里，不静默当成“没按键”。"""
    if _PYNPUT["loaded"]:
        return _PYNPUT["keyboard"]
    _PYNPUT["loaded"] = True
    if "DISPLAY" not in os.environ and "linux" in sys.platform:
        _PYNPUT["error"] = "没有 DISPLAY：pynput 监听在无 X 会话下不可用"
        return None
    try:
        from pynput import keyboard as _keyboard

        _PYNPUT["keyboard"] = _keyboard
    except Exception as exc:  # 缺依赖或后端不可用
        _PYNPUT["error"] = f"导入 pynput 失败：{exc}"
        logging.info(_PYNPUT["error"])
    return _PYNPUT["keyboard"]


def pynput_available() -> bool:
    return load_pynput() is not None


class URKeyboardTeleop(Teleoperator):
    """专为 UR 6 自由度机械臂定制的键盘遥操作控制器。

    两条输入通路：
      * 新通路（推荐）：会话持有 `KeyboardInputBackend`，把 `InputSnapshot` 交给
        `get_action_from_snapshot()`；本类只做“按键 → delta_*”的映射，不持有后端。
      * 旧通路（保留对照）：`connect()` 后由 pynput 监听填 `current_pressed`，
        `get_action()` 返回同样的 delta_*，供旧示例/对照实验使用；不是默认安全入口。
    """

    config_class = URKeyboardTeleopConfig
    name = "ur_keyboard"

    def __init__(self, config: URKeyboardTeleopConfig):
        super().__init__(config)
        self.config = config

        self.event_queue = Queue()
        self.misc_keys_queue = Queue()  # 用于记录如 回车、退格 等事件键
        self.current_pressed = {}
        self.listener = None
        self.logs = {}
        self._input_backend = None

    @property
    def action_features(self) -> dict[str, type]:
        """逐字段声明动作类型（不再用聚合的 dtype/shape/names）。"""
        features = {
            "delta_shoulder_pan": float,
            "delta_shoulder_lift": float,
            "delta_elbow": float,
            "delta_wrist_1": float,
            "delta_wrist_2": float,
            "delta_wrist_3": float,
            "delta_x": float,
            "delta_y": float,
            "delta_z": float,
            "delta_rx": float,
            "delta_ry": float,
            "delta_rz": float,
        }
        if self.config.use_gripper:
            features["gripper"] = float
        return features

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        keyboard = _PYNPUT["keyboard"]
        return (
            keyboard is not None
            and self.listener is not None
            and isinstance(self.listener, keyboard.Listener)
            and self.listener.is_alive()
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    @property
    def has_input_backend(self) -> bool:
        return self._input_backend is not None

    def set_input_backend(self, backend: Any) -> None:
        """注入会话持有的输入后端（不产生 I/O，也不启动它）。"""
        for method in ("start", "poll", "close"):
            if not callable(getattr(backend, method, None)):
                raise TypeError(f"输入后端缺少 {method}()，不符合 KeyboardInputBackendProtocol")
        self._input_backend = backend

    def connect(self, calibrate: bool = True) -> None:
        """`calibrate` 只是与官方设备接口对齐；键盘没有校准，参数被忽略。

        接受该参数不等于获得任何写设备/校准权限。
        """
        _ = calibrate  # 键盘无校准动作
        if self.is_connected:
            raise DeviceAlreadyConnectedError("UR Keyboard is already connected.")

        keyboard = load_pynput()
        if keyboard is None:
            raise ImportError(f"pynput 不可用，无法启动键盘监听（{_PYNPUT['error']}）")

        logging.info("UR 专属键盘监听已启动。")
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self.listener.start()

    def disconnect(self) -> None:
        if self.listener is not None:
            try:
                self.listener.stop()
            except Exception as exc:
                logging.warning("停止键盘监听失败：%s", exc)
        self.listener = None

    def calibrate(self) -> None:
        pass

    # ==========================================
    # 按键解析：pynput 的 Key/字符 → 统一按键 id
    # ==========================================
    def _get_key_id(self, key: Any) -> str | None:
        char = getattr(key, "char", None)
        if char:
            return char.lower() if char.isalpha() else char
        name = getattr(key, "name", None)
        if name:
            return name.lower()
        return None

    def _on_press(self, key: Any) -> None:
        key_id = self._get_key_id(key)
        if key_id is None:
            return
        self.event_queue.put((key_id, True))
        if key_id in ("enter", "backspace", "q"):
            self.misc_keys_queue.put(key_id)

    def _on_release(self, key: Any) -> None:
        key_id = self._get_key_id(key)
        if key_id is None:
            return
        self.event_queue.put((key_id, False))
        if key_id == "esc":
            logging.info("ESC pressed, disconnecting.")
            self.disconnect()

    def _drain_pressed_keys(self) -> None:
        while not self.event_queue.empty():
            key_id, is_pressed = self.event_queue.get_nowait()
            self.current_pressed[key_id] = is_pressed

    def configure(self) -> None:
        pass

    # ==========================================
    # 核心动作映射（唯一的 delta_* 生成处）
    # ==========================================
    def _deltas_from_pressed(self) -> dict[str, Any]:
        deltas = {
            "delta_shoulder_pan": 0.0,
            "delta_shoulder_lift": 0.0,
            "delta_elbow": 0.0,
            "delta_wrist_1": 0.0,
            "delta_wrist_2": 0.0,
            "delta_wrist_3": 0.0,
            "delta_x": 0.0,
            "delta_y": 0.0,
            "delta_z": 0.0,
            "delta_rx": 0.0,
            "delta_ry": 0.0,
            "delta_rz": 0.0,
        }
        gripper_action = 1.0
        j_step = self.config.joint_step_size
        t_step = self.config.cartesian_translation_step_size
        _ = self.config.cartesian_rotation_step_size  # 首版旋转为零，保留字段

        for key_id, is_pressed in self.current_pressed.items():
            if not is_pressed:
                continue
            if key_id in JOINT_KEYS:
                field, sign = JOINT_KEYS[key_id]
                deltas[field] = sign * j_step
            elif key_id in CARTESIAN_KEYS:
                field, sign = CARTESIAN_KEYS[key_id]
                deltas[field] = sign * t_step
            elif key_id == "space":
                # 旧通路里 Space 兼作夹爪开；UR 首版无末端夹爪，use_gripper 默认 False
                gripper_action = 2
            elif key_id in ("ctrl_l", "ctrl_r"):
                gripper_action = 0

        if self.config.use_gripper:
            deltas["gripper"] = gripper_action
        return deltas

    def get_action(self) -> dict[str, Any]:
        """旧通路：消费 pynput 队列后生成 delta_*（保留给对照/旧示例）。"""
        before_read_t = time.perf_counter()
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "UR Keyboard is not connected. You need to run `connect()` before `get_action()`."
            )
        self._drain_pressed_keys()
        deltas = self._deltas_from_pressed()
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t
        return deltas

    def get_action_from_snapshot(self, snapshot: InputSnapshot) -> dict[str, Any]:
        """新通路：把一个有效 `InputSnapshot` 映射成 delta_*。

        无效快照（失焦/超时/查询失败/关窗）一律拒绝：此时该由会话撤权并请求停止，
        不能把“空键集”当成操作者正常松开。
        """
        if not getattr(snapshot, "valid", False):
            raise ValueError(
                "输入快照无效（reason=%r），不能生成动作；应撤权并请求停止"
                % getattr(snapshot, "reason", None)
            )
        before_read_t = time.perf_counter()
        self.current_pressed = {key_id: True for key_id in snapshot.pressed_keys}
        deltas = self._deltas_from_pressed()
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t
        return deltas

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    # ==========================================
    # 数据集录制事件检测
    # ==========================================
    def get_teleop_events(self) -> dict[str, Any]:
        if not self.is_connected:
            return {
                TeleopEvents.IS_INTERVENTION: False,
                TeleopEvents.TERMINATE_EPISODE: False,
                TeleopEvents.SUCCESS: False,
                TeleopEvents.RERECORD_EPISODE: False,
            }

        # 检测是否有动作键被按下（标志着人类正在干预/遥操作）
        is_intervention = any(self.current_pressed.get(key_id, False) for key_id in ACTION_KEY_IDS)

        terminate_episode = False
        success = False
        rerecord_episode = False

        while not self.misc_keys_queue.empty():
            key_id = self.misc_keys_queue.get_nowait()
            if key_id == "enter":
                success = True
            elif key_id == "backspace":
                terminate_episode = True
                rerecord_episode = True
            elif key_id == "q":
                terminate_episode = True
                success = False

        return {
            TeleopEvents.IS_INTERVENTION: is_intervention,
            TeleopEvents.TERMINATE_EPISODE: terminate_episode,
            TeleopEvents.SUCCESS: success,
            TeleopEvents.RERECORD_EPISODE: rerecord_episode,
        }
