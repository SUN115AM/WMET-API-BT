from __future__ import annotations

import time

from .discovery import RECEIVER_DESCRIPTION, discover_devices, list_serial_ports
from .models import DeviceConfig, Pose
from .tracker import INIT_POSE_COMMAND, TrackerManager


class WemtBtAPI:
    """Simple Python API for reading WEMT tracking data over a Bluetooth receiver."""

    def __init__(
        self,
        port: str | None = None,
        device_id: str = "receiver-1",
        baudrate: int = 115200,
        devices: list[DeviceConfig] | None = None,
        smoothing_window: int = 8,
        trajectory_length: int = 35,
        send_start: bool = True,
        module_name: str | None = None,
    ) -> None:
        if devices is None:
            if port is None:
                # No port given: use every connected WEMT-BT receiver.
                devices = discover_devices(baudrate=baudrate)
                if not devices:
                    raise RuntimeError(
                        "No WEMT-BT receiver found (USB description "
                        f"{RECEIVER_DESCRIPTION!r}). Serial ports seen: {list_serial_ports()}. "
                        "Plug in the receiver or pass port='COMx'."
                    )
            else:
                devices = [DeviceConfig(id=device_id, port=port, baudrate=baudrate)]

        self.default_device_id = devices[0].id
        self.devices = devices
        self.smoothing_window = smoothing_window
        self.trajectory_length = trajectory_length
        self.send_start = send_start
        self.module_name = module_name
        self.trackers = TrackerManager()

    def start_tracking(self) -> list[Pose]:
        return self.trackers.start(
            self.devices,
            smoothing_window=self.smoothing_window,
            trajectory_length=self.trajectory_length,
            send_start=self.send_start,
            module_name=self.module_name,
        )

    def stop_tracking(self) -> list[str]:
        return self.trackers.stop()

    def poses(self) -> list[Pose]:
        return self.trackers.poses()

    def module_ids(self) -> list[int]:
        return sorted({pose.module_id for pose in self.poses()})

    def pose(self, module_id: int | None = None, device_id: str | None = None) -> Pose:
        if module_id is None:
            poses = [pose for pose in self.poses() if device_id in (None, pose.device_id)]
            if not poses:
                raise KeyError("No WEMT module has been seen yet.")
            return poses[0]

        pose = self.trackers.pose(module_id, device_id)
        if pose is None:
            raise KeyError(f"No pose received yet for WEMT module {module_id}.")
        return pose

    def wait_for_pose(
        self,
        module_id: int | None = None,
        device_id: str | None = None,
        timeout_s: float = 5.0,
    ) -> Pose:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                pose = self.pose(module_id, device_id)
            except KeyError:
                pose = None
            if pose and pose.connected and pose.timestamp is not None:
                return pose
            time.sleep(0.02)

        errors = [tracker.error for tracker in self.trackers.trackers() if tracker.error]
        detail = f" Receiver error: {errors[0]}" if errors else ""
        target = "any module" if module_id is None else f"module {module_id}"
        raise TimeoutError(f"No live pose received for {target} within {timeout_s} seconds.{detail}")

    def init_pose(self, device_id: str | None = None) -> None:
        """Send 'init pose' to the receiver (same as the Init Pose button in the PyQt app)."""
        self.send_command(INIT_POSE_COMMAND, device_id)

    def send_command(self, command: bytes | str, device_id: str | None = None) -> None:
        """Send a raw text command such as 'mag start' to a receiver."""
        if isinstance(command, str):
            command = command.encode("ascii")
        if not command.endswith(b"\r\n"):
            command += b"\r\n"

        device_id = device_id or self.default_device_id
        tracker = self.trackers.tracker(device_id)
        if tracker is None:
            raise KeyError(f"Unknown or stopped WEMT-BT receiver: {device_id}")
        tracker.write(command)
