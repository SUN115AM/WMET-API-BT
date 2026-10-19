import time

import pytest

pytest.importorskip("serial")

from wemt_bt_api import WemtBtAPI
from wemt_bt_api.models import DeviceConfig
from wemt_bt_api.packet import POSITION_RANGE_M, QUATERNION_RANGE, build_packet, compress
from wemt_bt_api.tracker import START_COMMAND, DeviceTracker


def make_packet(wire_id, note, x_m):
    p_raw = [compress(v, *POSITION_RANGE_M) for v in (x_m, 0.0, 0.0)]
    q_raw = [compress(v, *QUATERNION_RANGE) for v in (1.0, 0.0, 0.0, 0.0)]
    return build_packet(wire_id, 77, note, [(note * 30 + k, p_raw, q_raw) for k in range(3)])


def wait_until(condition, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.01)
    return condition()


def test_api_over_loopback():
    # loop:// echoes everything written, so writing packets simulates the receiver.
    api = WemtBtAPI(port="loop://", send_start=False, smoothing_window=1)
    try:
        api.start_tracking()
        tracker = api.trackers.tracker(api.default_device_id)
        assert wait_until(lambda: tracker.connected)
        tracker.write(make_packet(8, 1, 0.5) + make_packet(3, 2, -0.25))

        pose2 = api.wait_for_pose(module_id=2, timeout_s=2)
        pose3 = api.wait_for_pose(module_id=3, timeout_s=2)
        assert pose2.wire_id == 8
        assert pose2.battery == 77
        assert pose2.position_mm[0] == pytest.approx(500.0, abs=0.1)
        assert pose3.position_mm[0] == pytest.approx(-250.0, abs=0.1)
        assert api.module_ids() == [2, 3]
    finally:
        assert api.stop_tracking() == ["receiver-1"]


def test_start_command_echo_is_skipped():
    tracker = DeviceTracker(DeviceConfig(id="r", port="loop://"), smoothing_window=1)
    tracker.start()
    try:
        assert wait_until(lambda: tracker.connected)
        tracker.write(make_packet(1, 5, 0.1))
        assert wait_until(lambda: tracker.pose(1) is not None)
        assert tracker.pose(1).note == 5
        assert tracker.dropped_bytes == len(START_COMMAND)
    finally:
        tracker.stop()
    assert not tracker.pose(1).connected


def test_bad_port_reports_error():
    api = WemtBtAPI(port="/dev/does-not-exist-xyz", send_start=False)
    try:
        api.start_tracking()
        with pytest.raises(TimeoutError, match="Receiver error"):
            api.wait_for_pose(timeout_s=0.5)
    finally:
        api.stop_tracking()
