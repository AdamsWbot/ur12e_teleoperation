# src/core/master.py
import time
from src.common.types import (
    RawDeviceData,
    RobotState,
    JointState,
    Pose,
    MasterNormalizer,
)

class DefaultNormalizer(MasterNormalizer):
    """将 RawDeviceData 标准化为 RobotState（计划 §2.3）

    drop_index: 当 raw.joint 为 7 轴时，丢弃第几个关节（0-based，默认 6 = 第7关节）。
                由 S570Mapper 等异构设备映射器配置。
    """

    def __init__(self, drop_index: int = 6):
        self._drop_index = drop_index

    def normalize(self, raw: RawDeviceData) -> RobotState:
        # 1. 时间戳统一 — 使用 time.monotonic()
        timestamp = time.monotonic()

        # 2. 关节数据 — raw.joint 为 None 时补 (0.0,) * 6
        if raw.joint is None or len(raw.joint) == 0:
            joint = JointState(q=(0.0,) * 6)
        elif len(raw.joint) == 6:
            joint = JointState(q=tuple(raw.joint))
        elif len(raw.joint) == 7:
            # 异构设备（如 S570 有 7 轴）→ 丢弃指定关节，保留 6 轴
            q_list = list(raw.joint)
            dropped = q_list.pop(self._drop_index)
            joint = JointState(q=tuple(q_list))
        else:
            raise ValueError(
                f"raw.joint 应为 6 或 7 轴数据，实际收到 {len(raw.joint)} 轴: {raw.joint}"
            )

        # 3. TCP 位姿 — raw.tcp 为 None 时补 Pose(0,0,0,0,0,0)
        tcp = raw.tcp if raw.tcp is not None else Pose(0, 0, 0, 0, 0, 0)

        # 4. 构造并返回 RobotState
        return RobotState(
            timestamp=timestamp,
            joint=joint,
            tcp_pose=tcp,
        )