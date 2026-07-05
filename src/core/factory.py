from devices.ur12e import UR12eReader
from devices.s570 import S570Reader
from devices.keyboard import KeyboardReader
from src.core.mapper import IdentityMapper, S570Mapper, KeyboardMapper


class SystemFactory:
    def __init__(self, cfg: dict):
        self.cfg = cfg

    def create_device(self):
        device_type = self.cfg["device"]
        if device_type == "ur12e":
            return UR12eReader(self.cfg["master"])
        elif device_type == "s570":
            return S570Reader(self.cfg["s570"])
        elif device_type == "keyboard":
            return KeyboardReader(self.cfg["keyboard"])
        raise ValueError(f"Unknown device type: {device_type}")

    def create_mapper(self):
        device_type = self.cfg["device"]
        if device_type == "ur12e":
            return IdentityMapper()
        elif device_type == "s570":
            return S570Mapper()
        elif device_type == "keyboard":
            return KeyboardMapper()
        raise ValueError(f"Unknown device type: {device_type}")
