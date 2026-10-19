from __future__ import annotations

from collections import deque
import copy
import threading

import numpy as np

from .math_utils import make_homogeneous_matrix
from .models import DeviceConfig, Pose, utc_now
from .packet import WemtBtPacket, WemtBtPacketStream

try:
    import serial  # pip install pyserial
except ImportError:  # packet parsing still works without pyserial
    serial = None


START_COMMAND = b"mag start\r\n"
STOP_COMMAND = b"mag stop\r\n"
INIT_POSE_COMMAND = b"init pose\r\n"
READ_TIMEOUT_S = 0.1
# Line-idle time after which a complete packet at the end of the buffer is
# accepted (~35 byte times at 115200 baud; packets arrive every ~30 ms).
IDLE_GAP_S = 0.003


def set_name_command(name: str) -> bytes:
    return f"set name {name}\r\n".encode("ascii")


class _ModuleState:
    def __init__(self, smoothing_window: int, trajectory_length: int) -> None:
        self.positions = deque(maxlen=smoothing_window)
        self.trajectory = deque(maxlen=trajectory_length)
        self.pose: Pose | None = None


class DeviceTracker:
    """Reads one WEMT-BT receiver serial port and keeps the latest pose per module."""

    def __init__(
        self,
        config: DeviceConfig,
        smoothing_window: int = 8,
        trajectory_length: int = 35,
        send_start: bool = True,
        module_name: str | None = None,
    ) -> None:
        self.config = config
        self.smoothing_window = smoothing_window
        self.trajectory_length = trajectory_length
        self.send_start = send_start
        self.module_name = module_name
        self._modules: dict[int, _ModuleState] = {}
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial = None
        self.connected = False
        self.error: str | None = None
        self.dropped_bytes = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        if serial is None:
            raise RuntimeError("pyserial is not installed: pip install pyserial")
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name=f"wemt-bt-{self.config.id}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        with self._lock:
            self.connected = False
            for state in self._modules.values():
                if state.pose:
                    state.pose.connected = False

    def write(self, command: bytes) -> None:
        """Send a raw command (e.g. b"init pose\\r\\n") to the receiver."""
        with self._write_lock:
            if self._serial is None or not self._serial.is_open:
                raise RuntimeError(f"WEMT-BT receiver {self.config.id} is not connected.")
            self._serial.write(command)

    def snapshot(self) -> list[Pose]:
        with self._lock:
            return [copy.deepcopy(state.pose) for _, state in sorted(self._modules.items()) if state.pose]

    def pose(self, module_id: int) -> Pose | None:
        with self._lock:
            state = self._modules.get(module_id)
            return copy.deepcopy(state.pose) if state and state.pose else None

    def _open(self):
        return serial.serial_for_url(self.config.port, baudrate=self.config.baudrate, timeout=READ_TIMEOUT_S)

    def _run(self) -> None:
        stream = WemtBtPacketStream()
        try:
            ser = self._open()
            if self.module_name:
                # Send the name, then reopen the port before streaming.
                ser.write(set_name_command(self.module_name))
                ser.close()
                if self._stop_event.wait(1):
                    return
                ser = self._open()

            with self._write_lock:
                self._serial = ser
            ser.reset_input_buffer()
            if self.send_start:
                if self._stop_event.wait(1):
                    return
                self.write(START_COMMAND)

            with self._lock:
                self.connected = True
                self.error = None

            while not self._stop_event.is_set():
                chunk = ser.read(max(1, ser.in_waiting))
                packets = stream.feed(chunk) if chunk else []
                if stream.pending:
                    # A real packet is followed by a gap; a false match is
                    # followed immediately by more bytes.
                    ser.timeout = IDLE_GAP_S
                    try:
                        more = ser.read(max(1, ser.in_waiting))
                    finally:
                        ser.timeout = READ_TIMEOUT_S
                    packets += stream.feed(more) if more else stream.flush()
                for packet in packets:
                    self._apply_packet(packet)
                self.dropped_bytes = stream.dropped_bytes

        except Exception as exc:  # serial.SerialException, OSError, ...
            with self._lock:
                self.error = str(exc)
        finally:
            self._close()
            with self._lock:
                self.connected = False
                for state in self._modules.values():
                    if state.pose:
                        state.pose.connected = False
                        state.pose.error = self.error

    def _close(self) -> None:
        with self._write_lock:
            ser, self._serial = self._serial, None
        if ser is None:
            return
        try:
            if self.send_start and ser.is_open:
                ser.write(STOP_COMMAND)
        except Exception:
            pass
        finally:
            try:
                ser.close()
            except Exception:
                pass

    def _apply_packet(self, packet: WemtBtPacket) -> None:
        state = self._modules.get(packet.module_id)
        if state is None:
            state = _ModuleState(self.smoothing_window, self.trajectory_length)

        for frame in packet.frames:
            state.positions.append(frame.position_mm)
        smooth_position = np.mean(np.array(state.positions), axis=0)
        state.trajectory.append(smooth_position)

        frame = packet.frames[-1]
        transform = make_homogeneous_matrix(frame.quaternion_wxyz, smooth_position)
        pose = Pose(
            device_id=self.config.id,
            module_id=packet.module_id,
            connected=True,
            wire_id=packet.wire_id,
            battery=packet.battery,
            note=packet.note,
            timestamp=frame.timestamp,
            position_mm=smooth_position.tolist(),
            quaternion_wxyz=frame.quaternion_wxyz.tolist(),
            euler_xyz_degrees=frame.euler_xyz_degrees.tolist(),
            transform_matrix=transform.tolist(),
            trajectory_mm=[point.tolist() for point in state.trajectory],
            updated_at=utc_now(),
            error=None,
        )

        with self._lock:
            state.pose = pose
            self._modules[packet.module_id] = state


class TrackerManager:
    def __init__(self) -> None:
        self._trackers: dict[str, DeviceTracker] = {}
        self._lock = threading.Lock()

    def start(
        self,
        configs: list[DeviceConfig],
        smoothing_window: int = 8,
        trajectory_length: int = 35,
        send_start: bool = True,
        module_name: str | None = None,
    ) -> list[Pose]:
        with self._lock:
            for config in configs:
                existing = self._trackers.pop(config.id, None)
                if existing:
                    existing.stop()

                tracker = DeviceTracker(
                    config,
                    smoothing_window=smoothing_window,
                    trajectory_length=trajectory_length,
                    send_start=send_start,
                    module_name=module_name,
                )
                self._trackers[config.id] = tracker
                tracker.start()

            return [pose for tracker in self._trackers.values() for pose in tracker.snapshot()]

    def stop(self, device_id: str | None = None) -> list[str]:
        with self._lock:
            ids = [device_id] if device_id else list(self._trackers)
            stopped: list[str] = []
            for current_id in ids:
                tracker = self._trackers.pop(current_id, None)
                if tracker:
                    tracker.stop()
                    stopped.append(current_id)
            return stopped

    def poses(self) -> list[Pose]:
        with self._lock:
            return [pose for tracker in self._trackers.values() for pose in tracker.snapshot()]

    def pose(self, module_id: int, device_id: str | None = None) -> Pose | None:
        with self._lock:
            if device_id is None:
                trackers = list(self._trackers.values())
            else:
                trackers = [self._trackers[device_id]] if device_id in self._trackers else []
            for tracker in trackers:
                pose = tracker.pose(module_id)
                if pose is not None:
                    return pose
            return None

    def tracker(self, device_id: str) -> DeviceTracker | None:
        with self._lock:
            return self._trackers.get(device_id)

    def trackers(self) -> list[DeviceTracker]:
        with self._lock:
            return list(self._trackers.values())
