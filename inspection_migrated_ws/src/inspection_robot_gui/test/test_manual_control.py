from inspection_robot_gui.config import (
    DEFAULT_ANGULAR_SPEED_RADPS,
    DEFAULT_LINEAR_SPEED_MPS,
)
from inspection_robot_gui.main_window import MainWindow, manual_control_block_reason


def _reason(**overrides):
    values = {
        "launch_running": True,
        "actuation_enabled": True,
        "motor_pair": "cd",
        "teleop_subscribers": 1,
        "safety_level": "SAFE",
        "safety_age": 0.05,
        "scan_age": 0.05,
    }
    values.update(overrides)
    return manual_control_block_reason(**values)


def test_manual_control_reports_missing_remote_bringup_first():
    assert "开始建图" in _reason(launch_running=False)


def test_manual_control_requires_explicit_motor_enable():
    assert "未启用电机输出" in _reason(actuation_enabled=False)
    assert "未启用电机输出" in _reason(motor_pair="disabled")


def test_manual_control_requires_gate_safety_and_scan():
    assert "/cmd_vel_teleop" in _reason(teleop_subscribers=0)
    assert "安全状态" in _reason(safety_age=1.0)
    assert "FAULT" in _reason(safety_level="FAULT")
    assert "雷达扫描" in _reason(scan_age=1.0)


def test_manual_control_ready_and_uses_raised_track_speed_defaults():
    assert _reason() == ""
    assert DEFAULT_LINEAR_SPEED_MPS == 0.02
    assert DEFAULT_ANGULAR_SPEED_RADPS == 0.10


def test_hold_command_republishes_until_release_then_sends_zero():
    published = []

    class Timer:
        started = False

        def start(self):
            self.started = True

        def stop(self):
            self.started = False

    class Ros:
        def publish_cmd_vel(self, linear_x, angular_z):
            published.append((linear_x, angular_z))

    class Harness:
        _start_manual_command = MainWindow._start_manual_command
        _publish_manual_command = MainWindow._publish_manual_command
        _stop_manual_command = MainWindow._stop_manual_command

        def _begin_manual_takeover(self):
            return True

    window = Harness()
    window.ros = Ros()
    window.manual_command_timer = Timer()
    window._manual_command = (0.0, 0.0)

    assert window._start_manual_command(0.02, 0.10)
    window._publish_manual_command()
    assert window.manual_command_timer.started
    assert published == [(0.02, 0.10), (0.02, 0.10)]

    window._stop_manual_command()
    assert not window.manual_command_timer.started
    assert published[-1] == (0.0, 0.0)


def test_blocked_command_only_publishes_a_safe_zero():
    published = []

    class Timer:
        started = False

        def start(self):
            self.started = True

        def stop(self):
            self.started = False

    class Ros:
        def publish_cmd_vel(self, linear_x, angular_z):
            published.append((linear_x, angular_z))

    class Harness:
        _start_manual_command = MainWindow._start_manual_command
        _publish_manual_command = MainWindow._publish_manual_command
        _stop_manual_command = MainWindow._stop_manual_command

        def _begin_manual_takeover(self):
            return False

    window = Harness()
    window.ros = Ros()
    window.manual_command_timer = Timer()
    window._manual_command = (0.0, 0.0)

    assert not window._start_manual_command(0.02, 0.0)
    assert not window.manual_command_timer.started
    assert published == [(0.0, 0.0)]
