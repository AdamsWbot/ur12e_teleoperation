# P2 交付物（Slave端：UR12e 控制层）

## 一、核心代码

### 1. src/core/slave.py

```python
from __future__ import annotations

import logging
from src.common.types import RobotCommand, SlaveConfig

try:
    from ur_rtde import RTDEControlInterface as RTDEControl
except ImportError:
    RTDEControl = None


class SlaveController:
    def __init__(self, cfg: SlaveConfig):
        self.cfg = cfg
        self.rtde = None
        self._connected = False
        self._last_q = None
        self.logger = logging.getLogger("SlaveController")

    def connect(self) -> bool:
        if RTDEControl is None:
            return False

        try:
            self.rtde = RTDEControl(self.cfg.ip)
            self._connected = True
            return True
        except Exception:
            return False

    def execute(self, command: RobotCommand) -> bool:
        if not self._connected:
            return False

        q = list(command.joint.q)

        # basic clamp
        if self._last_q:
            for i in range(6):
                if abs(q[i] - self._last_q[i]) > 0.3:
                    q[i] = self._last_q[i] + 0.3 * (1 if q[i] > self._last_q[i] else -1)

        try:
            self.rtde.servoJ(q, self.cfg.servoj_lookahead_time, self.cfg.servoj_gain)
            self._last_q = q
            return True
        except Exception:
            return False

    def stop(self):
        if self.rtde:
            try:
                self.rtde.stopScript()
            except:
                pass

    @property
    def is_connected(self):
        return self._connected
```

---

## 二、测试文件

### 2. src/tests/test_rtde.py

```python
import unittest
from unittest.mock import MagicMock

from src.core.slave import SlaveController
from src.common.types import RobotCommand, JointState, SlaveConfig


class TestRTDE(unittest.TestCase):

    def setUp(self):
        self.cfg = SlaveConfig(
            ip="127.0.0.1",
            rtde_frequency=125,
            servoj_lookahead_time=0.03,
            servoj_gain=300,
            max_retries=3,
            retry_interval=1.0
        )

    def test_execute(self):
        ctrl = SlaveController(self.cfg)

        mock = MagicMock()
        ctrl.rtde = mock
        ctrl._connected = True

        cmd = RobotCommand(
            timestamp=0.0,
            joint=JointState(q=(0,0,0,0,0,0)),
            delta=JointState(q=(0,0,0,0,0,0))
        )

        ok = ctrl.execute(cmd)

        self.assertTrue(ok)
        mock.servoJ.assert_called_once()

    def test_disconnect(self):
        ctrl = SlaveController(self.cfg)
        ctrl.rtde = MagicMock()
        ctrl._connected = True

        ctrl.stop()
        ctrl.rtde.stopScript.assert_called_once()
```

---

## 三、依赖

### requirements.txt

```
ur-rtde
numpy
```

---

## 四、运行脚本

### run/run_slave.py

```python
import time
from src.core.slave import SlaveController
from src.common.types import RobotCommand, JointState, SlaveConfig


def main():
    cfg = SlaveConfig(
        ip="127.0.0.1",
        rtde_frequency=125,
        servoj_lookahead_time=0.03,
        servoj_gain=300,
        max_retries=3,
        retry_interval=1.0
    )

    slave = SlaveController(cfg)
    slave.connect()

    while True:
        cmd = RobotCommand(
            timestamp=time.time(),
            joint=JointState(q=(0,0,0,0,0,0)),
            delta=JointState(q=(0,0,0,0,0,0))
        )

        slave.execute(cmd)
        time.sleep(0.01)
```

---

## 五、接口说明（docs/p2_slave.md）

### 输入
RobotCommand:
- joint.q: 6维关节角
- delta.q: 关节变化量

### 输出
- execute() -> bool

### 控制映射
RobotCommand → UR RTDE servoJ

---

## 六、验收标准

- [ ] connect 成功
- [ ] servoJ 正常调用
- [ ] execute 返回 True
- [ ] mock test 可运行
