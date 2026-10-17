# 临时契约件：仅补齐 P2 消费的 InputSnapshot（P1 交付后必须替换/合并）
#
# 背景：计划 §3.4 规定 `src/lerobot/ur_teleop_contracts.py`（N1）由 P1 独占写入，
# 里面包含 InputSnapshot / LeaderSample / RobotSnapshot / Candidate / MotionPermit /
# SessionReport 等共享类型与全部校验。当前部署里 N1 尚未交付（团队 ur/main 未发布），
# 而 P2 的 `input_backend.py` 必须返回共享的 InputSnapshot。本文件因此只放
# **InputSnapshot 一个类型**，字段按 plans/task-cards.md §1 与计划 §3.4 的描述排列，
# 不含任何校验函数，不导入业务模块，也不代表接口冻结。
#
# 处理规则：
#   1. 本文件不是 N1。P1 交付 N1 后，直接用它替换本文件（同名同字段即可，P2 代码不用改）；
#   2. 后续若 N1 的字段名/语义与这里不同，以 N1 为准，P2 调整消费点，不在这里私改字段；
#   3. 不要把其他共享类型（LeaderSample、Candidate 等）加进本文件来“凑齐”契约。

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

__temp_contract__ = True  # 标记：等 P1 的 N1 交付后删除

FocusState = Literal["yes", "no", "unknown"]


@dataclass(frozen=True)
class InputSnapshot:
    """一次 poll 得到的完整键盘状态。

    字段语义（与计划 §3.4、task-cards §1 对齐）：
        backend_epoch   后端启动代次；start() 换一次，旧代次的快照立即作废。
        seq             同一代次内单调递增的样本号，用于发现乱序/重放。
        sample_mono_ns  采样时刻（主机单调时钟，time.monotonic_ns）。
        valid_until_ns  本样本的有效期截止（同一主机单调时钟）。
        pressed_keys    完整按键集合；失效快照里是“最后一次成功读取的键集”，只作诊断，
                        不能当作正常释放，也不能据此继续运行。
        focus           当前焦点在本后端窗口：yes / no / unknown。
        valid           本次快照能否作为运行依据。
        reason          失效原因（有效时为 None）：focus_lost / window_closed /
                        query_failed / late / deadline_exceeded / not_started …
    """

    backend_epoch: int
    seq: int
    sample_mono_ns: int
    valid_until_ns: int
    pressed_keys: frozenset[str]
    focus: FocusState
    valid: bool
    reason: str | None = None
