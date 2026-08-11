"""Unit tests for robot_control_ui.remote_health."""

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from robot_control_ui.logic.remote_health import (
    SystemHealth,
    classify_ssh_error,
    compute_cpu_percent,
    parse_loadavg,
    parse_meminfo,
    parse_proc_stat_cpu,
    parse_temp,
    parse_throttled,
    parse_uptime,
    undervoltage_now,
    undervoltage_seen,
)


def test_parse_temp_millidegree():
    assert parse_temp("39500") == 39.5
    assert parse_temp("51000\n") == 51.0
    assert parse_temp("") is None
    assert parse_temp("abc") is None


def test_parse_meminfo_percent():
    mem = "MemTotal:       1000 kB\nMemFree:        100 kB\nMemAvailable:   300 kB\n"
    assert parse_meminfo(mem) == 70.0
    assert parse_meminfo("") is None
    assert parse_meminfo("MemTotal: 0 kB") is None


def test_parse_proc_stat_cpu_and_percent():
    sample = (
        "cpu  100 0 50 800 10 0 0 0 0 0\n"
        "cpu0 50 0 25 400 5 0 0 0 0 0\n"
        "intr 12345\n"
    )
    prev = parse_proc_stat_cpu(sample)
    assert prev == (100, 0, 50, 800, 10, 0, 0)
    curr = (110, 0, 60, 830, 10, 0, 0)
    assert compute_cpu_percent(prev, curr) == 40.0
    assert compute_cpu_percent(None, curr) is None


def test_parse_loadavg_and_uptime():
    assert parse_loadavg("0.10 0.05 0.01 1/100 1234") == 0.10
    assert parse_loadavg("") is None
    assert parse_uptime("12345.67 99999.99") == 12345.67
    assert parse_uptime("") is None


def test_parse_throttled_and_flags():
    assert parse_throttled("throttled=0x50000") == 0x50000
    assert parse_throttled("0x1") == 0x1
    assert parse_throttled("") is None
    assert parse_throttled("throttled=0x0") == 0x0
    assert undervoltage_now(0x1) is True
    assert undervoltage_now(0x0) is False
    assert undervoltage_seen(0x10000) is True
    assert undervoltage_seen(0x1) is False
    assert undervoltage_now(None) is False


def test_classify_ssh_error():
    assert classify_ssh_error("Permission denied (publickey).") == "auth_failed"
    assert classify_ssh_error("Connection refused") == "refused"
    assert classify_ssh_error("Connection timed out") == "timeout"
    assert classify_ssh_error("Host key verification failed.") == "hostkey_changed"
    assert classify_ssh_error("Could not resolve hostname pi") == "host_unreachable"
    assert classify_ssh_error("") == "unknown"


def test_system_health_expired_markers():
    health = SystemHealth()
    assert health.online is False
    assert health.error_code == "unprobed"
    assert health.expired == {}
