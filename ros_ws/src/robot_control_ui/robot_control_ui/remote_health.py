"""SSH-based health probing for the robot.

Runs ``ssh -o BatchMode=yes`` in a background thread so the TK main
loop never blocks.  Values keep their last good reading and are marked
``expired`` instead of being zeroed when the link drops.

Voltage truth boundary: ``vcgencmd measure_volts`` reports the CPU
core rail, NOT the 5V input.  Input voltage stays ``None`` (N/A)
unless a real ADC/telemetry path supplies it.
"""

import shlex
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Optional


SSH_BASE = [
    "ssh",
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=2",
    "-o", "StrictHostKeyChecking=accept-new",
]

REMOTE_PROBE_CMD = (
    "cat /sys/class/thermal/thermal_zone0/temp; "
    "cat /proc/meminfo; "
    "cat /proc/stat; "
    "cat /proc/loadavg; "
    "cat /proc/uptime; "
    "vcgencmd get_throttled 2>/dev/null; "
    "vcgencmd measure_volts core 2>/dev/null"
)


# ---------------------------------------------------------------------------
# Pure parsing helpers (unit-testable)
# ---------------------------------------------------------------------------

def parse_temp(text: str):
    """Parse a millidegree temperature value into Celsius."""
    try:
        value = float(text.strip())
    except (TypeError, ValueError):
        return None
    return round(value / 1000.0, 1)


def parse_meminfo(text: str):
    """Return used-percent from /proc/meminfo content."""
    total = available = None
    for line in text.splitlines():
        if line.startswith("MemTotal:"):
            total = float(line.split()[1])
        elif line.startswith("MemAvailable:"):
            available = float(line.split()[1])
    if total is None or available is None or total <= 0:
        return None
    return round(100.0 * (total - available) / total, 1)


def parse_proc_stat_cpu(text: str):
    """Return (user, nice, system, idle, iowait, irq, softirq) of the first cpu line."""
    for line in text.splitlines():
        if line.startswith("cpu "):
            parts = line.split()
            if len(parts) < 5:
                return None
            return (int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]),
                    int(parts[5]) if len(parts) > 5 else 0,
                    int(parts[6]) if len(parts) > 6 else 0,
                    int(parts[7]) if len(parts) > 7 else 0)
    return None


def compute_cpu_percent(prev, curr):
    """CPU busy-percent between two /proc/stat samples."""
    if prev is None:
        return None
    prev_total = sum(prev)
    curr_total = sum(curr)
    delta_total = curr_total - prev_total
    delta_idle = (curr[3] + curr[4]) - (prev[3] + prev[4])
    if delta_total <= 0:
        return None
    return round(100.0 * (delta_total - delta_idle) / delta_total, 1)


def parse_loadavg(text: str):
    try:
        return float(text.split()[0])
    except (IndexError, ValueError):
        return None


def parse_uptime(text: str):
    try:
        return float(text.split()[0])
    except (IndexError, ValueError):
        return None


def parse_throttled(text: str):
    """Parse ``throttled=0x...`` into raw flags; None when unsupported."""
    text = text.strip()
    if not text:
        return None
    marker = "throttled="
    if marker in text:
        text = text.split(marker, 1)[1]
    try:
        return int(text, 16) if text.lower().startswith("0x") else int(text)
    except ValueError:
        return None


def undervoltage_now(flags):
    return bool(flags is not None and (flags & 0x1))


def undervoltage_seen(flags):
    return bool(flags is not None and (flags & 0x10000))


def classify_ssh_error(stderr_text: str) -> str:
    """Map ssh stderr to a stable error code."""
    low = (stderr_text or "").lower()
    if "could not resolve hostname" in low:
        return "host_unreachable"
    if "permission denied" in low or "authentication failed" in low:
        return "auth_failed"
    if "host key verification failed" in low:
        return "hostkey_changed"
    if "connection refused" in low:
        return "refused"
    if "timed out" in low or "connection timed out" in low:
        return "timeout"
    if "connection reset" in low:
        return "reset"
    return "unknown"


@dataclass
class SystemHealth:
    online: bool = False
    error_code: str = "unprobed"
    latency_ms: float = None
    last_success: float = None
    temp_c: float = None
    cpu_percent: float = None
    mem_percent: float = None
    load_1m: float = None
    uptime_s: float = None
    throttled_flags: int = None
    core_voltage_v: float = None
    expired: dict = None  # metric name -> True when stale

    def __post_init__(self):
        if self.expired is None:
            self.expired = {}


# ---------------------------------------------------------------------------
# Background probe
# ---------------------------------------------------------------------------

class RemoteHealthProbe:
    """Periodically SSH-probes the robot in a daemon thread.

    ``callback(health: SystemHealth)`` is invoked from the probe thread;
    the UI must marshal it to the main thread (e.g. a queue).
    """

    def __init__(self, user, host, interval=2.0, callback=None, stop_event=None):
        self.user = user
        self.host = host
        self.interval = interval
        self.callback = callback
        self.stop_event = stop_event or threading.Event()
        self._thread = None
        self._prev_stat = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.stop_event.set()

    def probe_once(self):
        health = SystemHealth()
        cmd = SSH_BASE + ["%s@%s" % (self.user, self.host), REMOTE_PROBE_CMD]
        started = time.time()
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
        except subprocess.TimeoutExpired:
            health.error_code = "timeout"
            return self._expire(health)
        latency_ms = round((time.time() - started) * 1000.0, 1)
        if proc.returncode != 0:
            health.error_code = classify_ssh_error(proc.stderr)
            return self._expire(health)

        sections = proc.stdout.splitlines()
        health.online = True
        health.error_code = "ok"
        health.latency_ms = latency_ms
        health.last_success = time.time()
        if sections:
            health.temp_c = parse_temp(sections[0])

        # locate blocks by marker lines
        lines = proc.stdout.splitlines()
        idx = {}
        for i, line in enumerate(lines):
            if line.startswith("MemTotal:"):
                idx["mem"] = i
            elif line.startswith("cpu "):
                idx["stat"] = i
            elif "loadavg" not in line and len(line.split()) >= 3 and idx.get("stat") is not None and i > idx.get("stat", 0):
                # loadavg line: 3 floats + 3 ids
                pass
        # simpler: find markers sequentially
        def find_marker(marker):
            for i, line in enumerate(lines):
                if marker in line:
                    return i
            return None

        mem_i = find_marker("MemTotal:")
        stat_i = find_marker("cpu ")
        load_i = find_marker("/proc/loadavg") if find_marker("/proc/loadavg") is not None else None

        mem_block = chr(10).join(lines[mem_i:stat_i]) if mem_i is not None and stat_i is not None and stat_i > mem_i else ""
        health.mem_percent = parse_meminfo(mem_block) if mem_block else None

        # cpu: everything from stat line until loadavg values
        if stat_i is not None:
            stat_block_lines = []
            for line in lines[stat_i:]:
                if line.startswith("cpu"):
                    stat_block_lines.append(line)
                else:
                    break
            stat_block = chr(10).join(stat_block_lines)
            curr = parse_proc_stat_cpu(stat_block)
            health.cpu_percent = compute_cpu_percent(self._prev_stat, curr) if curr else None
            if curr is not None:
                self._prev_stat = curr

        loadavg_line = None
        for line in lines:
            if line.startswith("cpu") or line.startswith("Mem") or "loadavg" in line:
                continue
            parts = line.split()
            if len(parts) >= 3:
                try:
                    float(parts[0])
                    float(parts[1])
                    float(parts[2])
                    loadavg_line = line
                    break
                except ValueError:
                    pass
        health.load_1m = parse_loadavg(loadavg_line) if loadavg_line else None

        uptime_line = None
        for line in lines:
            parts = line.split()
            if len(parts) == 2 and parts[1] == "sec" and not line.startswith("cpu"):
                uptime_line = line
                break
        # uptime is two numbers on one line: seconds + idle seconds
        if uptime_line is None:
            for line in lines:
                parts = line.split()
                if len(parts) == 2:
                    try:
                        float(parts[0])
                        float(parts[1])
                        uptime_line = line
                        break
                    except ValueError:
                        pass
        health.uptime_s = parse_uptime(uptime_line) if uptime_line else None

        throttled_line = None
        for line in lines:
            if "throttled=" in line:
                throttled_line = line
                break
        health.throttled_flags = parse_throttled(throttled_line) if throttled_line else None

        voltage_line = None
        for line in lines:
            if line.startswith("volt="):
                voltage_line = line
                break
        if voltage_line:
            try:
                health.core_voltage_v = round(float(voltage_line.split("=")[1].rstrip("V")), 4)
            except (IndexError, ValueError):
                health.core_voltage_v = None
        return health

    def _expire(self, health):
        for name in ("temp_c", "cpu_percent", "mem_percent", "load_1m",
                     "uptime_s", "throttled_flags", "core_voltage_v"):
            if getattr(health, name) is None:
                health.expired[name] = True
        return health

    def _loop(self):
        while not self.stop_event.is_set():
            health = self.probe_once()
            if self.callback:
                try:
                    self.callback(health)
                except Exception:
                    pass
            self.stop_event.wait(self.interval)
