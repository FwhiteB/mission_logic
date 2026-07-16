"""UI-level receiver interface contracts.

This module intentionally contains only observations and controls that a
mission controller can get from an RD8200-style receiver. Simulation-only truth
such as nearest pipeline geometry belongs in field simulation, not here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class ReceiverPosition(Protocol):
    """Minimal receiver pose shape accepted by receiver adapters."""

    x: float
    y: float
    z: float
    receiver_yaw_degrees: float


@dataclass(frozen=True)
class ReceiverPose:
    """Receiver pose with instrument yaw independent from robot body yaw."""

    x: float
    y: float
    z: float
    receiver_yaw_degrees: float = 0.0


@dataclass(frozen=True)
class ReceiverStatus:
    """Basic receiver health and configuration state."""

    mode: str
    frequency_hz: float
    gain_level: float
    healthy: bool
    battery_percent: float | None = None
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class ReceiverReading:
    """UI-level receiver reading exposed to mission logic."""

    position: ReceiverPosition
    signal_strength: float
    compass_heading_degrees: float
    receiver_yaw_degrees: float
    alignment_error_degrees: float | None
    depth_m: float | None
    current_value: float | None
    arrow_hint: str | None
    valid_depth: bool
    valid_current: bool
    mode: str
    frequency_hz: float


class ReceiverInterface(Protocol):
    """Minimal receiver API used by mission logic."""

    @property
    def status(self) -> ReceiverStatus:
        """Return current receiver health and configuration."""

    def set_mode(self, mode: str) -> None:
        """Switch antenna/location mode, for example peak or peak_plus."""

    def set_frequency(self, frequency_hz: float) -> None:
        """Switch receiver frequency."""

    def adjust_gain(self, delta: float) -> float:
        """Apply a receiver gain adjustment and return the new level."""

    def read(self, position: ReceiverPosition) -> ReceiverReading:
        """Read UI-level observations at the supplied receiver position."""
