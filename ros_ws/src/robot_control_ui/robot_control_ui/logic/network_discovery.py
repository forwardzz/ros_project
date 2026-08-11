"""LAN device discovery for the robot control UI.

Pure-logic helpers (no Qt / rclpy dependency) so the scanning
strategy can be unit tested.  Best-effort discovery: devices that
block ICMP and are absent from the neighbour table may not be found.
"""

import ipaddress
import socket
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import List, Optional


MAX_HOSTS = 254


@dataclass
class DeviceInfo:
    ip: str
    hostname: str = "--"
    mac: str = "--"
    reachable: bool = False
    ssh_open: bool = False
    current: bool = False


def is_valid_ipv4(text: str) -> bool:
    """Strict IPv4 validation (rejects port suffixes and ranges)."""
    if not isinstance(text, str):
        return False
    text = text.strip()
    try:
        addr = ipaddress.IPv4Address(text)
    except ipaddress.AddressValueError:
        return False
    return str(addr) == text


def subnet_to_host_range(subnet: str, max_hosts: int = MAX_HOSTS) -> List[str]:
    """Expand ``192.168.43.0/24`` into host addresses to probe.

    Subnets larger than /24 are capped at ``max_hosts`` so scanning a
    /16 cannot hang the UI for minutes.
    """
    network = ipaddress.ip_network(subnet, strict=False)
    hosts = [str(ip) for ip in network.hosts()]
    if len(hosts) > max_hosts:
        hosts = hosts[:max_hosts]
    return hosts


def read_neighbor_table() -> dict:
    """Parse /proc/net/arp into {ip: mac}. Returns {} on non-Linux."""
    table: dict = {}
    try:
        with open("/proc/net/arp", "r", encoding="utf-8") as fh:
            lines = fh.readlines()
    except OSError:
        return table
    for line in lines[1:]:
        parts = line.split()
        if len(parts) >= 4 and parts[3] != "00:00:00:00:00:00":
            table[parts[0]] = parts[3]
    return table


def _resolve_hostname(ip: str) -> str:
    try:
        return socket.gethostbyaddr(ip)[0]
    except OSError:
        return "--"


def _probe_ssh(ip: str, timeout: float) -> bool:
    try:
        with socket.create_connection((ip, 22), timeout=timeout):
            return True
    except OSError:
        return False


def scan_subnet(
    subnet: str,
    timeout: float = 1.0,
    max_hosts: int = MAX_HOSTS,
    cancel_event: Optional[threading.Event] = None,
    current_ip: Optional[str] = None,
) -> List[DeviceInfo]:
    """Probe a subnet best-effort: neighbour table first, then TCP/ICMP.

    Returns DeviceInfo rows sorted by IP.  ``current_ip`` marks the
    currently selected robot address with ``current=True``.
    """
    hosts = subnet_to_host_range(subnet, max_hosts)
    neighbors = read_neighbor_table()
    devices: List[DeviceInfo] = []
    lock = threading.Lock()

    def probe(ip: str) -> Optional[DeviceInfo]:
        if cancel_event is not None and cancel_event.is_set():
            return None
        mac = neighbors.get(ip, "--")
        ssh_open = _probe_ssh(ip, timeout)
        if mac != "--" or ssh_open:
            return DeviceInfo(
                ip=ip,
                hostname=_resolve_hostname(ip),
                mac=mac,
                reachable=mac != "--",
                ssh_open=ssh_open,
                current=ip == current_ip,
            )
        return None

    with ThreadPoolExecutor(max_workers=min(32, len(hosts) or 1)) as pool:
        futures = [pool.submit(probe, ip) for ip in hosts]
        for future in as_completed(futures):
            if cancel_event is not None and cancel_event.is_set():
                break
            result = future.result()
            if result is not None:
                with lock:
                    devices.append(result)

    devices.sort(key=lambda d: ipaddress.IPv4Address(d.ip))
    return devices


def default_subnet() -> Optional[str]:
    """Determine the active subnet from the default route (Linux only)."""
    try:
        out = subprocess.run(
            ["ip", "route", "show", "default"],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in out.splitlines():
        parts = line.split()
        if "via" in parts and "dev" in parts:
            dev_index = parts.index("dev")
            if dev_index + 1 < len(parts):
                dev = parts[dev_index + 1]
                break
    else:
        return None
    try:
        out = subprocess.run(
            ["ip", "-4", "addr", "show", "dev", dev],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in out.splitlines():
        if "inet " in line:
            parts = line.split()
            cidr_index = parts.index("inet") + 1
            if cidr_index < len(parts):
                network = ipaddress.ip_network(parts[cidr_index], strict=False)
                return str(network)
    return None
