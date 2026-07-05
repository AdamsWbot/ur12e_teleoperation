from devices.ur12e import UR12eReader
from devices.s570 import S570Reader
from devices.keyboard import KeyboardReader
from src.common.config import AppConfig
from src.core.mapper import IdentityMapper, S570Mapper, KeyboardMapper


class SystemFactory:
    def __init__(self, cfg: AppConfig):
        self.cfg = cfg

    def create_device(self):
        t = self.cfg.device
        if t == "ur12e":
            return UR12eReader(self.cfg.master)
        elif t == "s570":
            return S570Reader(self.cfg.s570)
        elif t == "keyboard":
            return KeyboardReader(self.cfg.keyboard)
        raise ValueError(f"Unknown device type: {t}")

    def create_mapper(self):
        t = self.cfg.device
        if t == "ur12e":
            return IdentityMapper()
        elif t == "s570":
            return S570Mapper()
        elif t == "keyboard":
            return KeyboardMapper()
        raise ValueError(f"Unknown device type: {t}")
