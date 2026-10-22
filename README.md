# WEMT-BT API

Simple Python API for WEMT electromagnetic tracking modules connected over
Bluetooth (WEMT-BT). It follows the same layout and calls as the TCP
`wemt_api` package.

## Install

```powershell
python -m venv .venv
.\.venv\Scripts\python -m pip install -e ".[dev]"
```

## Basic Usage

```python
from wemt_bt_api import WemtBtAPI

api = WemtBtAPI()                                  # finds the receiver's COM port
# api = WemtBtAPI(port="COM3")                     # or force a port

try:
    api.start_tracking()
    pose = api.wait_for_pose(timeout_s=5)          # first module seen
    # pose = api.wait_for_pose(module_id=2)        # or a specific module

    print("Module:", pose.module_id, "battery:", pose.battery)
    print("Position mm:", pose.position_mm)
    print("Euler xyz degrees:", pose.euler_xyz_degrees)
    print("Quaternion wxyz:", pose.quaternion_wxyz)
    print("All modules:", api.module_ids())
finally:
    api.stop_tracking()
```

Other calls:

- `api.poses()`: latest pose of every module seen so far
- `api.init_pose()`: send `init pose` (same as the Init Pose button in the PyQt app)
- `api.send_command("mag stop")`: send any raw text command
- `WemtBtAPI(port=..., send_start=False)`: don't send `mag start` / `mag stop` (device already streaming)
- `WemtBtAPI(port=..., module_name="0026")`: send bytes `set name 0026\r\n` first for initialization

Finding the receiver's COM port:

`WemtBtAPI()` without `port` uses every serial port whose USB description is
"nRF52 USB CDC" (one tracker per receiver) and raises `RuntimeError` listing
the visible ports if none is found. To inspect ports yourself:

```python
from wemt_bt_api.discovery import find_receiver_ports, list_serial_ports

print(list_serial_ports())
print(find_receiver_ports())
```

## Development Notes

Core files:

- `examples/wemt_bt_tracking_example.py`: start here (finds the receiver automatically)
- `wemt_bt_api/api.py`: simple start/stop/read API
- `wemt_bt_api/packet.py`: WEMT-BT 59-byte packet parser with resync
- `wemt_bt_api/tracker.py`: serial connection and live pose tracking per module
- `wemt_bt_api/discovery.py`: find the receiver's COM port

Run the tests with `python -m pytest`. The tracker tests use pyserial's
`loop://` port, so no hardware is needed.
