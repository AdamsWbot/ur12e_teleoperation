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
    """将 RawDeviceData 标准化为 RobotState（计划 §2.3）"""
    
    def normalize(self, raw: RawDeviceData) -> RobotState:
        # 1. 时间戳统一 — 使用 time.monotonic()
        timestamp = time.monotonic()
        
        # 2. 关节数据 — raw.joint 为 None 时补 (0.0,) * 6
        if raw.joint is None or len(raw.joint) == 0:
            joint = JointState(q=(0.0,) * 6)
        elif len(raw.joint) != 6:
            raise ValueError(
                f"raw.joint 应为 6 轴数据，实际收到 {len(raw.joint)} 轴: {raw.joint}"
            )
        else:
            joint = JointState(q=tuple(raw.joint))
        
        # 3. TCP 位姿 — raw.tcp 为 None 时补 Pose(0,0,0,0,0,0)
        tcp = raw.tcp if raw.tcp is not None else Pose(0, 0, 0, 0, 0, 0)
        
        # 4. 构造并返回 RobotState
        return RobotState(
            timestamp=timestamp,
            joint=joint,
            tcp_pose=tcp,
        )