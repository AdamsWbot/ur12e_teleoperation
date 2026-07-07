from abc import ABC, abstractmethod
from queue import Queue

from src.common.types import RobotCommand, RobotState


class Bridge(ABC):
    """Communication abstraction between pipeline stages."""

    @abstractmethod
    def send_state(self, state: RobotState) -> None:
        ...

    @abstractmethod
    def receive_state(self) -> RobotState:
        ...

    @abstractmethod
    def send_command(self, command: RobotCommand) -> None:
        ...

    @abstractmethod
    def receive_command(self) -> RobotCommand:
        ...


class LocalBridge(Bridge):
    """In-process bridge backed by queues for local simulation/tests."""

    def __init__(self) -> None:
        self._state_queue: Queue[RobotState] = Queue()
        self._command_queue: Queue[RobotCommand] = Queue()

    def send_state(self, state: RobotState) -> None:
        self._state_queue.put(state)

    def receive_state(self) -> RobotState:
        return self._state_queue.get()

    def send_command(self, command: RobotCommand) -> None:
        self._command_queue.put(command)

    def receive_command(self) -> RobotCommand:
        return self._command_queue.get()


class TCPBridge(Bridge):
    """Reserved interface for future master/slave network deployment."""

    def send_state(self, state: RobotState) -> None:
        raise NotImplementedError("TCPBridge is reserved for future implementation")

    def receive_state(self) -> RobotState:
        raise NotImplementedError("TCPBridge is reserved for future implementation")

    def send_command(self, command: RobotCommand) -> None:
        raise NotImplementedError("TCPBridge is reserved for future implementation")

    def receive_command(self) -> RobotCommand:
        raise NotImplementedError("TCPBridge is reserved for future implementation")
