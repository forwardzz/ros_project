"""Unit tests for scripts/robot_ip_detect.py.

Runs the detection-helper pure logic (interface parsing, host matching,
robot selection) without touching the live network.
"""

import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[4] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import robot_ip_detect as rid  # noqa: E402


def test_usable_interfaces_parses_ip_addr_output(monkeypatch):
    fake = (
        "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536\n"
        "    inet 127.0.0.1/8 scope host lo\n"
        "3: eth0: <BROADCAST,MULTICAST,UP> mtu 1500\n"
        "    inet 169.254.190.41/16 scope global noprefixroute eth0\n"
        "    inet 26.237.243.202/8 scope global noprefixroute eth0\n"
        "4: eth1: <BROADCAST,MULTICAST,UP> mtu 1500\n"
        "    inet 192.168.1.178/24 scope global noprefixroute eth1\n"
        "5: eth2: <BROADCAST,MULTICAST,UP> mtu 1400\n"
        "    inet 192.168.43.20/24 scope global noprefixroute eth2\n"
    )
    import subprocess

    monkeypatch.setattr(
        rid.subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a[0], 0, stdout=fake, stderr=""),
    )
    ifaces = rid.usable_interfaces()
    assert ("eth1", "192.168.1.178", "192.168.1.0/24") in ifaces
    assert ("eth2", "192.168.43.20", "192.168.43.0/24") in ifaces
    # link-local / Radmin / loopback excluded
    assert all(ip != "169.254.190.41" for _, ip, _ in ifaces)
    assert all(ip != "26.237.243.202" for _, ip, _ in ifaces)


def test_wsl_ip_for_host_picks_same_subnet(monkeypatch):
    monkeypatch.setattr(
        rid,
        "usable_interfaces",
        lambda: [
            ("eth1", "192.168.1.178", "192.168.1.0/24"),
            ("eth2", "192.168.43.20", "192.168.43.0/24"),
        ],
    )
    assert rid.wsl_ip_for_host("192.168.43.30") == "192.168.43.20"
    assert rid.wsl_ip_for_host("192.168.1.5") == "192.168.1.178"


def test_wsl_ip_for_host_falls_back_to_first(monkeypatch):
    monkeypatch.setattr(
        rid,
        "usable_interfaces",
        lambda: [("eth1", "192.168.1.178", "192.168.1.0/24")],
    )
    # host on a different subnet -> fallback to first usable interface
    assert rid.wsl_ip_for_host("10.9.9.9") == "192.168.1.178"


class _FakeDev:
    def __init__(self, ip, ssh_open=False, reachable=False):
        self.ip = ip
        self.hostname = ip
        self.ssh_open = ssh_open
        self.reachable = reachable


def test_detect_selects_unique_ssh_ok_robot(monkeypatch):
    monkeypatch.setattr(
        rid,
        "usable_interfaces",
        lambda: [("eth2", "192.168.43.20", "192.168.43.0/24")],
    )
    monkeypatch.setattr(
        rid,
        "scan_subnet",
        lambda subnet, timeout=0.6: [
            _FakeDev(ip="192.168.43.1"),
            _FakeDev(ip="192.168.43.30", reachable=True),
        ],
    )
    monkeypatch.setattr(rid, "passwordless_ssh_ok", lambda user, ip: ip == "192.168.43.30")
    result = rid.detect("yy")
    assert result["robot"] == "192.168.43.30"
    assert [c["ip"] for c in result["candidates"]] == ["192.168.43.30"]


def test_detect_no_robot_when_no_ssh_ok(monkeypatch):
    monkeypatch.setattr(
        rid,
        "usable_interfaces",
        lambda: [("eth1", "192.168.1.178", "192.168.1.0/24")],
    )
    monkeypatch.setattr(
        rid,
        "scan_subnet",
        lambda subnet, timeout=0.6: [],
    )
    monkeypatch.setattr(rid, "passwordless_ssh_ok", lambda user, ip: False)
    result = rid.detect("yy")
    assert result["robot"] == ""


def test_fast_check_exit_codes(monkeypatch):
    monkeypatch.setattr(rid, "passwordless_ssh_ok", lambda user, ip, timeout=3.0: ip == "192.168.43.30")
    with pytest.raises(SystemExit) as ok:
        rid.main([
            "fast-check", "--ip", "192.168.43.30", "--user", "yy",
        ])
    assert ok.value.code == 0
    with pytest.raises(SystemExit) as bad:
        rid.main([
            "fast-check", "--ip", "192.168.43.99", "--user", "yy",
        ])
    assert bad.value.code == 1
