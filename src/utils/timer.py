import time


class Rate:
    """Fixed-frequency loop helper similar to rospy.Rate."""

    def __init__(self, hz: float) -> None:
        if hz <= 0:
            raise ValueError(f"rate must be positive, got {hz}")
        self._period = 1.0 / hz
        self._next_time = time.monotonic() + self._period

    def sleep(self) -> None:
        now = time.monotonic()
        remaining = self._next_time - now

        if remaining > 0:
            time.sleep(remaining)
            self._next_time += self._period
            return

        # If the loop overruns, resynchronize to avoid accumulating delay.
        self._next_time = now + self._period
