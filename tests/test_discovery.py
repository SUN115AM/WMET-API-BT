import platform

import pytest

from wemt_bt_api import WemtBtAPI, api as api_module
from wemt_bt_api.discovery import parse_com_names, read_windows_bus_descriptions
from wemt_bt_api.models import DeviceConfig


def test_parse_com_names():
    assert parse_com_names("USB Serial Device (COM3)\nOther (com12)\n") == ["COM3", "COM12"]


@pytest.mark.skipif(platform.system().lower() != "windows", reason="PowerShell lookup is Windows only")
def test_description_is_not_executed(tmp_path):
    marker = tmp_path / "injected.txt"
    hostile = f'x" ) {{ }} }}; New-Item -ItemType File "{marker}"; ForEach-Object {{ if ("'
    assert read_windows_bus_descriptions(hostile) == []
    assert not marker.exists()


def test_api_discovers_port_when_none_given(monkeypatch):
    found = [DeviceConfig(id="receiver-1", port="COM7")]
    monkeypatch.setattr(api_module, "discover_devices", lambda baudrate: found)
    assert WemtBtAPI().devices == found


def test_api_reports_missing_receiver(monkeypatch):
    monkeypatch.setattr(api_module, "discover_devices", lambda baudrate: [])
    monkeypatch.setattr(api_module, "list_serial_ports", lambda: [("COM1", "Communications Port")])
    with pytest.raises(RuntimeError, match="COM1"):
        WemtBtAPI()
