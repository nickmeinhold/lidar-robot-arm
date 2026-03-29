"""Arm controller abstraction.

``ArmController`` defines the async interface that all controllers implement.
``ConsoleArmController`` prints joint state to the terminal for testing
without hardware.
"""

from __future__ import annotations

import sys
import time
from abc import ABC, abstractmethod

from .protocol import ArmAngles, TrackingStatus


class ArmController(ABC):
    """Base class for arm controllers (console, servo, sim, etc.)."""

    @abstractmethod
    async def update(self, angles: ArmAngles, tracking: TrackingStatus) -> None:
        """Apply new joint state to the arm."""

    @abstractmethod
    async def stop(self) -> None:
        """Safely stop the arm (e.g. hold position or go limp)."""


class ConsoleArmController(ArmController):
    """Prints joint state to stdout, rate-limited to ~10 Hz.

    Uses ``\\r`` carriage return to overwrite the line in-place, giving
    a nice live-updating display without scrolling.
    """

    MIN_INTERVAL = 0.1  # 10 Hz max refresh

    def __init__(self) -> None:
        self._last_print: float = 0.0

    async def update(self, angles: ArmAngles, tracking: TrackingStatus) -> None:
        now = time.monotonic()
        if now - self._last_print < self.MIN_INTERVAL:
            return
        self._last_print = now

        line = f"\r{angles.degrees_str()} | {tracking}"
        sys.stdout.write(line)
        sys.stdout.flush()

    async def stop(self) -> None:
        # Move to a new line so the prompt doesn't clobber the last status
        sys.stdout.write("\n")
        sys.stdout.flush()
