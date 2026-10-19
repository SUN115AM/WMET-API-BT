from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Sequence

import numpy as np

from .math_utils import make_homogeneous_matrix, quaternion_to_euler


# 59-byte Bluetooth packet (little-endian, packed):
#   head 0xAA, module_id, bat_vol, 3 x (uint32 t, int16 p[3], int16 q[4]), note, tail 0x0A
PACKET_HEAD = 0xAA
PACKET_END = 0x0A
FRAME_FMT = "I3h4h"
PACKET_FMT = "<BBB" + FRAME_FMT * 3 + "BB"
PACKET_SIZE = struct.calcsize(PACKET_FMT)
FRAMES_PER_PACKET = 3
_PACKET = struct.Struct(PACKET_FMT)

# Fixed-point ranges used by the firmware
POSITION_RANGE_M = (-2.0, 2.0)
QUATERNION_RANGE = (-1.0, 1.0)
POSITION_SCALE_MM = 1000.0
INT16_MIN, INT16_MAX = -32768, 32767
INT16_SPAN = 65535.0

# Wire ids 7..10 are alternate ids for the same physical rigid bodies.
RIGID_ID_MAP = {1: 1, 2: 2, 8: 2, 3: 3, 7: 3, 4: 4, 5: 5, 9: 5, 6: 6, 10: 6}


def decompress(value: int, low: float, high: float) -> float:
    """int16 fixed-point -> float."""
    value = min(max(value, INT16_MIN), INT16_MAX)
    return low + (value - INT16_MIN) / INT16_SPAN * (high - low)


def compress(value: float, low: float, high: float) -> int:
    """float -> int16 fixed-point (inverse of decompress)."""
    raw = round((value - low) / (high - low) * INT16_SPAN + INT16_MIN)
    return min(max(raw, INT16_MIN), INT16_MAX)


def rigid_id(wire_id: int) -> int:
    """Physical module id for a wire id (unknown ids are returned unchanged)."""
    return RIGID_ID_MAP.get(wire_id, wire_id)


@dataclass(frozen=True)
class WemtBtFrame:
    timestamp: int
    position_mm: np.ndarray
    quaternion_wxyz: np.ndarray
    euler_xyz_degrees: np.ndarray
    transform_matrix: np.ndarray


@dataclass(frozen=True)
class WemtBtPacket:
    wire_id: int
    module_id: int
    battery: int
    note: int
    frames: list[WemtBtFrame]


def _make_frame(timestamp: int, p_raw: Sequence[int], q_raw: Sequence[int]) -> WemtBtFrame:
    position_mm = np.array([decompress(v, *POSITION_RANGE_M) for v in p_raw]) * POSITION_SCALE_MM
    quaternion = np.array([decompress(v, *QUATERNION_RANGE) for v in q_raw])
    return WemtBtFrame(
        timestamp=timestamp,
        position_mm=position_mm,
        quaternion_wxyz=quaternion,
        euler_xyz_degrees=quaternion_to_euler(quaternion),
        transform_matrix=make_homogeneous_matrix(quaternion, position_mm),
    )


def parse_packet(packet: bytes, offset: int = 0) -> WemtBtPacket:
    """Parse one WEMT-BT 59-byte packet."""
    if len(packet) - offset < PACKET_SIZE:
        raise ValueError(f"WEMT-BT packet must be {PACKET_SIZE} bytes, got {len(packet) - offset}.")
    if packet[offset] != PACKET_HEAD:
        raise ValueError("WEMT-BT packet head must be 0xaa.")
    if packet[offset + PACKET_SIZE - 1] != PACKET_END:
        raise ValueError("WEMT-BT packet terminator must be 0x0a.")

    values = _PACKET.unpack_from(packet, offset)
    frames = []
    for i in range(FRAMES_PER_PACKET):
        base = 3 + i * 8
        frames.append(_make_frame(values[base], values[base + 1:base + 4], values[base + 4:base + 8]))

    wire_id = values[1]
    return WemtBtPacket(
        wire_id=wire_id,
        module_id=rigid_id(wire_id),
        battery=values[2],
        note=values[-2],
        frames=frames,
    )


def build_packet(
    wire_id: int,
    battery: int,
    note: int,
    frames: Sequence[tuple[int, Sequence[int], Sequence[int]]],
) -> bytes:
    """Encode a packet from raw (timestamp, p_int16[3], q_int16[4]) frames. Used by tests."""
    if len(frames) != FRAMES_PER_PACKET:
        raise ValueError(f"A WEMT-BT packet carries exactly {FRAMES_PER_PACKET} frames.")
    values: list[int] = [PACKET_HEAD, wire_id, battery]
    for timestamp, p_raw, q_raw in frames:
        values.append(timestamp)
        values.extend(p_raw)
        values.extend(q_raw)
    values += [note, PACKET_END]
    return _PACKET.pack(*values)


class WemtBtPacketStream:
    """Incrementally extract WEMT-BT packets from serial byte chunks.

    A candidate (head 0xaa ... tail 0x0a) is accepted when the next byte is the
    next packet's head, which rejects false matches caused by 0xaa / 0x0a bytes
    inside the payload. A candidate at the very end of the buffer stays pending
    until more data arrives or flush() is called (the tracker calls it once the
    line goes idle).
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> list[WemtBtPacket]:
        self._buffer.extend(data)
        packets: list[WemtBtPacket] = []
        i, size = 0, len(self._buffer)

        while size - i >= PACKET_SIZE:
            if self._buffer[i] == PACKET_HEAD and self._buffer[i + PACKET_SIZE - 1] == PACKET_END:
                following = i + PACKET_SIZE
                if following == size:
                    break
                if self._buffer[following] == PACKET_HEAD:
                    packets.append(parse_packet(self._buffer, i))
                    i = following
                    continue
            i += 1
            self.dropped_bytes += 1

        del self._buffer[:i]
        return packets

    @property
    def pending(self) -> bool:
        """True if the buffer holds exactly one complete but unconfirmed packet."""
        return (
            len(self._buffer) == PACKET_SIZE
            and self._buffer[0] == PACKET_HEAD
            and self._buffer[-1] == PACKET_END
        )

    def flush(self) -> list[WemtBtPacket]:
        """Accept a pending packet. Call when no more bytes are coming right now."""
        if not self.pending:
            return []
        packet = parse_packet(self._buffer)
        self._buffer.clear()
        return [packet]
