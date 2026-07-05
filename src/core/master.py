from abc import ABC, abstractmethod

from src.common.types import RobotState


class MasterReader(ABC):
    """所有主端设备的抽象基类"""

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def read(self) -> RobotState: ...

    @property
    @abstractmethod
    def is_connected(self) -> bool: ...
