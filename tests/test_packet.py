import random
import struct

import numpy as np
import pytest

from wemt_bt_api.packet import (
    PACKET_SIZE,
    POSITION_RANGE_M,
    QUATERNION_RANGE,
    WemtBtPacketStream,
    build_packet,
    compress,
    parse_packet,
)


def make_frames(t0=0, position_m=(0.1, 0.0, -1.5), quaternion=(1.0, 0.0, 0.0, 0.0)):
    p_raw = [compress(v, *POSITION_RANGE_M) for v in position_m]
    q_raw = [compress(v, *QUATERNION_RANGE) for v in quaternion]
    return [(t0 + 10 * k, p_raw, q_raw) for k in range(3)]


def random_packets(count, seed=1):
    rng = random.Random(seed)
    packets = []
    for note in range(count):
        frames = [
            (
                note * 30 + k,
                [rng.randint(-32768, 32767) for _ in range(3)],
                [rng.randint(-32768, 32767) for _ in range(4)],
            )
            for k in range(3)
        ]
        packets.append(build_packet(3, 90, note, frames))
    return packets


def feed_all(stream, data, chunk):
    packets = []
    for k in range(0, len(data), chunk):
        packets.extend(stream.feed(data[k:k + chunk]))
    packets.extend(stream.flush())
    return packets


def test_packet_size_is_59():
    assert PACKET_SIZE == 59


def test_field_offsets_match_reader_layout():
    raw = build_packet(5, 200, 99, [(0x11111111 * (k + 1), (1, 2, 3), (4, 5, 6, 7)) for k in range(3)])
    assert raw[0] == 0xAA
    assert raw[1] == 5
    assert raw[2] == 200
    assert struct.unpack("<I", raw[21:25])[0] == 0x22222222
    assert struct.unpack("<I", raw[39:43])[0] == 0x33333333
    assert raw[57] == 99
    assert raw[58] == 0x0A


def test_parse_packet_scales_position_to_mm():
    packet = parse_packet(build_packet(8, 87, 42, make_frames(t0=1000)))

    assert (packet.wire_id, packet.module_id, packet.battery, packet.note) == (8, 2, 87, 42)
    assert [frame.timestamp for frame in packet.frames] == [1000, 1010, 1020]
    frame = packet.frames[0]
    np.testing.assert_allclose(frame.position_mm, [100.0, 0.0, -1500.0], atol=0.1)
    np.testing.assert_allclose(frame.quaternion_wxyz, [1.0, 0.0, 0.0, 0.0], atol=1e-4)
    np.testing.assert_allclose(frame.euler_xyz_degrees, [0.0, 0.0, 0.0], atol=0.01)
    np.testing.assert_allclose(frame.transform_matrix[:3, 3], frame.position_mm)


def test_values_are_signed():
    packet = parse_packet(build_packet(1, 0, 0, [(0, (0, -1, -32768), (32767, 0, 0, 0))] * 3))
    assert abs(packet.frames[0].position_mm[0]) < 0.1
    assert packet.frames[0].position_mm[2] == pytest.approx(-2000.0)


def test_parse_packet_rejects_bad_framing():
    raw = bytearray(build_packet(1, 0, 0, make_frames()))
    with pytest.raises(ValueError):
        parse_packet(bytes(raw[:-1]))
    raw[-1] = 0x00
    with pytest.raises(ValueError):
        parse_packet(bytes(raw))


def test_stream_every_chunk_size():
    packets = random_packets(20)
    data = b"".join(packets)
    for chunk in (1, 2, 7, 58, 59, 60, 128, len(data)):
        stream = WemtBtPacketStream()
        assert [p.note for p in feed_all(stream, data, chunk)] == list(range(20))
        assert stream.dropped_bytes == 0


def test_stream_recovers_after_noise():
    stream = WemtBtPacketStream()
    out = feed_all(stream, b"\x11\xaa\x22\xaa" + b"".join(random_packets(5)), 7)
    assert len(out) == 5
    assert stream.dropped_bytes == 4


def test_stream_recovers_after_dropped_byte():
    packets = random_packets(20)
    bad = bytearray(packets[5])
    del bad[30]
    data = b"".join(packets[:5]) + bytes(bad) + b"".join(packets[6:])
    out = feed_all(WemtBtPacketStream(), data, 13)
    assert [p.note for p in out] == [n for n in range(20) if n != 5]


def test_stream_rejects_shifted_false_match():
    # stray 0xaa + packet whose note == 0x0a looks like a packet shifted by one byte
    a = build_packet(1, 0, 1, make_frames())
    b = build_packet(1, 0, 0x0A, make_frames())
    c = build_packet(1, 0, 3, make_frames())
    for chunk in (1, 5, 59, 1000):
        out = feed_all(WemtBtPacketStream(), a + b"\xaa" + b + c + a, chunk)
        assert [p.note for p in out] == [1, 10, 3, 1]


def test_partial_packet_waits_for_next_head_or_flush():
    first, second = random_packets(2)
    stream = WemtBtPacketStream()
    assert stream.feed(first[:40]) == []
    assert stream.flush() == []
    assert stream.feed(first[40:]) == []
    assert stream.pending
    assert len(stream.feed(second[:1])) == 1
    assert stream.feed(second[1:]) == []
    assert len(stream.flush()) == 1
