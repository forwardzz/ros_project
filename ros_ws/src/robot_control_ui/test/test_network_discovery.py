"""Unit tests for robot_control_ui.network_discovery."""

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from robot_control_ui.logic.network_discovery import (
    MAX_HOSTS,
    DeviceInfo,
    is_valid_ipv4,
    read_neighbor_table,
    subnet_to_host_range,
)


def test_is_valid_ipv4():
    assert is_valid_ipv4("192.168.43.30")
    assert is_valid_ipv4("10.0.0.1")
    assert not is_valid_ipv4("192.168.43.300")
    assert not is_valid_ipv4("1.2.3")
    assert not is_valid_ipv4("192.168.43.30:22")
    assert not is_valid_ipv4("")
    assert not is_valid_ipv4("abc")
    assert not is_valid_ipv4(None)


def test_subnet_to_host_range_full_24():
    hosts = subnet_to_host_range("192.168.43.0/24")
    assert len(hosts) == 254
    assert hosts[0] == "192.168.43.1"
    assert hosts[-1] == "192.168.43.254"


def test_subnet_to_host_range_capped_on_large_subnet():
    # /16 would be 65534 hosts; must be capped at MAX_HOSTS (254)
    hosts = subnet_to_host_range("192.168.0.0/16")
    assert len(hosts) == MAX_HOSTS


def test_subnet_to_host_range_accepts_host_form():
    hosts = subnet_to_host_range("192.168.43.99/24")
    assert hosts[0] == "192.168.43.1"


def test_read_neighbor_table_never_raises():
    table = read_neighbor_table()
    assert isinstance(table, dict)
    for ip, mac in table.items():
        assert ":" in mac


def test_device_info_defaults():
    dev = DeviceInfo(ip="192.168.43.30")
    assert dev.hostname == "--"
    assert dev.mac == "--"
    assert dev.reachable is False
    assert dev.ssh_open is False
    assert dev.current is False
