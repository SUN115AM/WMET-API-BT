from wemt_bt_api import WemtBtAPI


# No port given: the receiver's COM port is found automatically.
# Pass port="COM3" to force a specific port.
api = WemtBtAPI()
print("Receiver port(s):", [device.port for device in api.devices])

try:
    api.start_tracking()
    pose = api.wait_for_pose(timeout_s=5)

    print("Module:", pose.module_id, "battery:", pose.battery, "note:", pose.note)
    print("Position mm:", pose.position_mm)
    print("Euler xyz degrees:", pose.euler_xyz_degrees)
    print("Quaternion wxyz:", pose.quaternion_wxyz)
finally:
    api.stop_tracking()
