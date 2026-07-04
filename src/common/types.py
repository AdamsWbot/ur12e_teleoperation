from dataclasses import dataclass

@dataclass
class Pose:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float