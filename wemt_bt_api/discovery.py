from __future__ import annotations

import os
import platform
import re
import subprocess

from .models import DeviceConfig


# USB description of the nRF52 Bluetooth receiver dongle.
RECEIVER_DESCRIPTION = "nRF52 USB CDC"
COM_RE = re.compile(r"\((COM\d+)\)", re.IGNORECASE)


def list_serial_ports() -> list[tuple[str, str]]:
    """Return (port, description) for every serial port pyserial can see."""
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("pyserial is not installed: pip install pyserial") from exc
    return [(port.device, port.description) for port in list_ports.comports()]


def find_receiver_ports(description: str = RECEIVER_DESCRIPTION) -> list[str]:
    """Find serial ports whose USB description matches the WEMT-BT receiver."""
    if platform.system().lower() == "windows":
        ports = read_windows_bus_descriptions(description)
        if ports:
            return ports

    pattern = description.lower()
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    found = []
    for port in list_ports.comports():
        text = " ".join(str(v) for v in (port.description, port.product, port.manufacturer) if v)
        if pattern in text.lower():
            found.append(port.device)
    return found


def discover_devices(description: str = RECEIVER_DESCRIPTION, baudrate: int = 115200) -> list[DeviceConfig]:
    return [
        DeviceConfig(id=f"receiver-{index + 1}", port=port, baudrate=baudrate)
        for index, port in enumerate(find_receiver_ports(description))
    ]


def read_windows_bus_descriptions(description: str) -> list[str]:
    """Match DEVPKEY_Device_BusReportedDeviceDesc, which pyserial does not expose on Windows.

    The description is passed through an environment variable and matched as a
    literal substring, so it is never parsed as PowerShell code.
    """
    command = (
        "$pattern = '*' + [System.Management.Automation.WildcardPattern]::Escape($env:WEMT_BT_DESCRIPTION) + '*'; "
        'Get-WMIObject Win32_PnPEntity | Where-Object { $_.name -match "COM" } | ForEach-Object { '
        '$p = $_.GetDeviceProperties("DEVPKEY_Device_BusReportedDeviceDesc"); '
        "if ($p.DeviceProperties.Data -like $pattern) { $_.Name } }"
    )
    completed = subprocess.run(
        ["powershell", "-NoProfile", "-Command", command],
        check=False,
        capture_output=True,
        text=True,
        env={**os.environ, "WEMT_BT_DESCRIPTION": description},
    )
    return parse_com_names(completed.stdout or "")


def parse_com_names(output: str) -> list[str]:
    return [match.group(1).upper() for match in COM_RE.finditer(output)]
