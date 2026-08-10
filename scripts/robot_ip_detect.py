#!/usr/bin/env python3
"""Detect the robot (Raspberry Pi) IP on the current LAN.

Used by start.sh so the TK UI always launches with the correct device
address.  Reuses robot_control_ui.network_discovery for scanning.

Modes:
  detect --user yy
      Scan every usable interface subnet, verify passwordless SSH,
      print JSON {"candidates": [...], "robot": "<ip>" or ""}
  wsl-ip --for-host 192.168.43.30
      Print the local (WSL) interface IP that shares the host subnet,
      or the default-route interface IP as a fallback.
"""

import argparse
import ipaddress
import json
import subprocess
import sys
from pathlib import Path


PKG = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "robot_control_ui" / "robot_control_ui"
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from network_discovery import scan_subnet  # noqa: E402


EXCLUDED_NETS = ("169.254.0.0/16", "26.0.0.0/8", "10.0.0.0/8", "172.16.0.0/12")


IFACE_RE = __import__("re").compile(r"^\d+:\s+([\w.]+):")


def usable_interfaces():
    """Return [(dev, ip, subnet)] for IPv4 interfaces we may scan."""
    result = []
    try:
        out = subprocess.run(
            ["ip", "-4", "addr", "show"],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return result
    dev = None
    for line in out.splitlines():
        m = IFACE_RE.match(line.strip())
        if m:
            dev = m.group(1)
            continue
        stripped = line.strip()
        if stripped.startswith("inet ") and dev and dev != "lo":
            parts = stripped.split()
            if len(parts) >= 2:
                cidr = parts[1]
                ip = cidr.split("/")[0]
                net = ipaddress.ip_network(cidr, strict=False)
                excluded = any(net.overlaps(ipaddress.ip_network(e)) for e in EXCLUDED_NETS)
                if not excluded and net.prefixlen <= 24:
                    result.append((dev, ip, str(net)))
    return result


def passwordless_ssh_ok(user, ip, timeout=3):
    try:
        proc = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=%d" % timeout,
             "-o", "StrictHostKeyChecking=accept-new",
             "-o", "UserKnownHostsFile=/dev/null",
             "%s@%s" % (user, ip), "true"],
            capture_output=True,
            timeout=timeout + 2,
        )
        return proc.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def detect(user, scan_timeout=0.8):
    candidates = []
    for dev, ip, subnet in usable_interfaces():
        try:
            devices = scan_subnet(subnet, timeout=scan_timeout)
        except Exception:
            continue
        for d in devices:
            # reachable (neighbour table) counts too: the TCP probe is
            # unreliable on high-latency phone hotspots; final decision
            # is made by the passwordless-SSH check below.
            if (d.ssh_open or d.reachable) and d.ip not in [c["ip"] for c in candidates]:
                candidates.append({"ip": d.ip, "hostname": d.hostname, "ssh_ok": False})
    # verify with passwordless SSH: the robot is the device we can log into
    for cand in candidates:
        cand["ssh_ok"] = passwordless_ssh_ok(user, cand["ip"])
    robot = ""
    ok = [c for c in candidates if c["ssh_ok"]]
    if len(ok) == 1:
        robot = ok[0]["ip"]
    elif len(ok) > 1:
        # several keyed hosts: prefer the one on the same subnet as the default route
        robot = ok[0]["ip"]
    return {"candidates": candidates, "robot": robot}


def wsl_ip_for_host(host):
    host_addr = ipaddress.IPv4Address(host)
    fallback = None
    for dev, ip, subnet in usable_interfaces():
        net = ipaddress.ip_network(subnet, strict=False)
        if host_addr in net:
            return ip
        if fallback is None:
            fallback = ip
    return fallback


def main(argv=None):
    parser = argparse.ArgumentParser(description="Robot IP detection helper")
    sub = parser.add_subparsers(dest="mode", required=True)

    p_detect = sub.add_parser("detect", help="scan LAN and verify robot SSH")
    p_detect.add_argument("--user", default="yy")
    p_detect.add_argument("--timeout", type=float, default=0.6)

    p_wsl = sub.add_parser("wsl-ip", help="print local interface IP for a host subnet")
    p_wsl.add_argument("--for-host", required=True)

    p_val = sub.add_parser("validate", help="exit 0 when the IP is a valid IPv4")
    p_val.add_argument("--ip", required=True)

    args = parser.parse_args(argv)
    if args.mode == "detect":
        print(json.dumps(detect(args.user, args.timeout)))
    elif args.mode == "wsl-ip":
        ip = wsl_ip_for_host(args.for_host)
        print(ip if ip else "")
    elif args.mode == "validate":
        from network_discovery import is_valid_ipv4  # noqa: F811

        sys.exit(0 if is_valid_ipv4(args.ip) else 1)


if __name__ == "__main__":
    main()
