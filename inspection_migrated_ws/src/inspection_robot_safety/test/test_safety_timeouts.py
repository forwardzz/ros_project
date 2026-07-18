import math

from geometry_msgs.msg import Twist

from inspection_robot_safety.safety_monitor import (
    DEFAULT_STATUS_PUBLISH_PERIOD_SEC,
    SafetyMonitor,
)
from inspection_robot_safety.velocity_safety_gate import (
    DEFAULT_GATE_PUBLISH_PERIOD_SEC,
    DEFAULT_INPUT_TIMEOUT_SEC,
    DEFAULT_SAFETY_STATUS_TIMEOUT_SEC,
    DEFAULT_SCAN_TIMEOUT_SEC,
    MAX_FAILSAFE_STOP_BUDGET_SEC,
    MAX_GATE_PUBLISH_PERIOD_SEC,
    VelocitySafetyGate,
)


class _Duration:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)


class _Time:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return _Duration(self.seconds - other.seconds)


class _Clock:
    def __init__(self, seconds):
        self.seconds = seconds

    def now(self):
        return _Time(self.seconds)


def _gate_at(now_sec, last_status_sec=None, last_scan_sec=None):
    gate = object.__new__(VelocitySafetyGate)
    gate.clock = _Clock(now_sec)
    gate.safety_time = (
        None if last_status_sec is None else _Time(last_status_sec)
    )
    gate.safety_status_timeout = DEFAULT_SAFETY_STATUS_TIMEOUT_SEC
    gate.scan_time = None if last_scan_sec is None else _Time(last_scan_sec)
    gate.scan_timeout = DEFAULT_SCAN_TIMEOUT_SEC
    return gate


def test_safety_heartbeat_budget_is_below_half_second():
    assert DEFAULT_STATUS_PUBLISH_PERIOD_SEC == 0.10
    assert DEFAULT_SAFETY_STATUS_TIMEOUT_SEC == 0.30
    assert DEFAULT_SAFETY_STATUS_TIMEOUT_SEC < 0.50


def test_missing_abort_service_does_not_block_safety_heartbeat():
    class _AbortClient:
        def __init__(self):
            self.called = False

        def service_is_ready(self):
            return False

        def call_async(self, _request):
            self.called = True
            raise AssertionError("Unavailable abort service must not be called")

    class _Logger:
        def warn(self, _message):
            pass

    monitor = object.__new__(SafetyMonitor)
    monitor.abort_client = _AbortClient()
    monitor.get_logger = lambda: _Logger()

    monitor._request_abort()

    assert not monitor.abort_client.called


def test_scan_watchdog_includes_timer_margin_below_stop_budget():
    assert DEFAULT_GATE_PUBLISH_PERIOD_SEC == 0.02
    assert DEFAULT_SCAN_TIMEOUT_SEC == 0.40
    assert (
        DEFAULT_SCAN_TIMEOUT_SEC + MAX_GATE_PUBLISH_PERIOD_SEC
        < MAX_FAILSAFE_STOP_BUDGET_SEC
    )
    assert _gate_at(1.39, last_scan_sec=1.0)._scan_fresh()
    assert not _gate_at(1.41, last_scan_sec=1.0)._scan_fresh()


def test_fail_safe_parameters_cannot_be_relaxed_above_hard_limits():
    class _Parameter:
        value = 10.0

    gate = object.__new__(VelocitySafetyGate)
    gate.get_parameter = lambda _name: _Parameter()

    assert gate._bounded_parameter(
        "input_timeout_sec", DEFAULT_INPUT_TIMEOUT_SEC, 0.05,
        DEFAULT_INPUT_TIMEOUT_SEC
    ) == DEFAULT_INPUT_TIMEOUT_SEC
    assert gate._bounded_parameter(
        "safety_status_timeout_sec", DEFAULT_SAFETY_STATUS_TIMEOUT_SEC, 0.10,
        DEFAULT_SAFETY_STATUS_TIMEOUT_SEC
    ) == DEFAULT_SAFETY_STATUS_TIMEOUT_SEC
    assert gate._bounded_parameter(
        "scan_timeout_sec", DEFAULT_SCAN_TIMEOUT_SEC, 0.05,
        DEFAULT_SCAN_TIMEOUT_SEC
    ) == DEFAULT_SCAN_TIMEOUT_SEC
    assert gate._bounded_parameter(
        "publish_period_sec", DEFAULT_GATE_PUBLISH_PERIOD_SEC, 0.005,
        MAX_GATE_PUBLISH_PERIOD_SEC
    ) == MAX_GATE_PUBLISH_PERIOD_SEC


def test_missing_or_stale_safety_heartbeat_fails_closed():
    assert not _gate_at(1.0, None)._safety_fresh()
    assert _gate_at(1.29, 1.0)._safety_fresh()
    assert not _gate_at(1.31, 1.0)._safety_fresh()


def test_gate_reports_each_fail_closed_reason():
    gate = object.__new__(VelocitySafetyGate)
    gate.fault = False
    gate.external_stop = False
    gate.require_fresh_scan = True
    gate._safety_fresh = lambda: False
    gate._scan_fresh = lambda: True
    assert gate._stop_reason() == "safety status missing or stale"

    gate._safety_fresh = lambda: True
    gate.fault = True
    assert gate._stop_reason() == "safety monitor fault"

    gate.fault = False
    gate.external_stop = True
    assert gate._stop_reason() == "safety stop asserted"

    gate.external_stop = False
    gate._scan_fresh = lambda: False
    assert gate._stop_reason() == "laser scan missing or stale"

    gate._scan_fresh = lambda: True
    assert gate._stop_reason() == ""


def test_velocity_sanitizer_rejects_nonfinite_and_clamps_limits():
    gate = object.__new__(VelocitySafetyGate)
    gate.max_linear_speed = 0.18
    gate.max_angular_speed = 0.55

    command = Twist()
    command.linear.x = 1.0
    command.angular.z = -2.0
    output = gate._sanitize(command)
    assert output.linear.x == 0.18
    assert output.angular.z == -0.55

    command.linear.x = math.nan
    output = gate._sanitize(command)
    assert output.linear.x == 0.0
    assert output.angular.z == 0.0
