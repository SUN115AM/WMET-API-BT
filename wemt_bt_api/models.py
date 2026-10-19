from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone


@dataclass(frozen=True)
class DeviceConfig:
    """Serial (Bluetooth) port for one WEMT-BT receiver.

    One receiver dongle forwards packets from several WEMT modules, so one
    DeviceConfig can produce several poses (one per module).
    """

    id: str
    port: str
    baudrate: int = 115200


@dataclass
class Pose:
    """Latest measured pose for one WEMT module seen by a WEMT-BT receiver."""

    device_id: str
    module_id: int
    connected: bool
    wire_id: int | None = None
    battery: int | None = None
    note: int | None = None
    timestamp: int | None = None
    position_mm: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion_wxyz: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    euler_xyz_degrees: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    transform_matrix: list[list[float]] = field(
        default_factory=lambda: [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    trajectory_mm: list[list[float]] = field(default_factory=list)
    updated_at: datetime | None = None
    error: str | None = None


def utc_now() -> datetime:
    return datetime.now(timezone.utc)
