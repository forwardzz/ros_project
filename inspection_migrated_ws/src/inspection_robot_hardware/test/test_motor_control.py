import math

import pytest

from inspection_robot_hardware.motor_control import MotorControllerCore


class FakeBoard:
    def __init__(self):
        self.commands = []
        self.stop_count = 0

    def drive(self, left, right):
        self.commands.append((left, right))

    def stop(self):
        self.stop_count += 1
        self.commands.append((0.0, 0.0))


def make_controller(board, now=0.0, **overrides):
    values = {
        "max_rpm": 80.0,
        "wheel_radius": 0.025,
        "track_width": 0.155,
        "min_breakout_pwm": 28.0,
        "max_pwm": 70.0,
        "max_linear_speed": 0.18,
        "max_angular_speed": 0.55,
        "max_linear_accel": 100.0,
        "max_angular_accel": 100.0,
        "control_period_sec": 0.05,
        "command_timeout_sec": 0.35,
        "now_sec": now,
    }
    values.update(overrides)
    return MotorControllerCore(board, **values)


@pytest.mark.parametrize(
    "linear,angular,left_sign,right_sign",
    [
        (0.1, 0.0, 1, 1),
        (-0.1, 0.0, -1, -1),
        (0.0, 0.3, -1, 1),
        (0.0, -0.3, 1, -1),
    ],
)
def test_differential_drive_direction(linear, angular, left_sign, right_sign):
    board = FakeBoard()
    controller = make_controller(board)
    controller.set_command(linear, angular, 0.0)
    controller.tick(0.01)
    left, right = board.commands[-1]

    assert math.copysign(1, left) == left_sign
    assert math.copysign(1, right) == right_sign


def test_velocity_rpm_and_pwm_are_limited():
    board = FakeBoard()
    controller = make_controller(board, max_rpm=10.0, max_pwm=55.0)
    controller.set_command(99.0, 99.0, 0.0)
    controller.tick(0.01)

    assert controller.target_vx == 0.18
    assert controller.target_vz == 0.55
    assert all(abs(value) <= 55.0 for value in board.commands[-1])


def test_watchdog_stops_after_035_seconds():
    board = FakeBoard()
    controller = make_controller(board)
    controller.set_command(0.1, 0.0, 0.0)
    controller.tick(0.10)
    assert board.commands[-1] != (0.0, 0.0)

    assert controller.watchdog(0.351)
    assert board.commands[-1] == (0.0, 0.0)
    assert controller.current_vx == 0.0


def test_invalid_command_stops_immediately():
    board = FakeBoard()
    controller = make_controller(board)
    controller.set_command(0.1, 0.0, 0.0)
    controller.tick(0.1)

    with pytest.raises(ValueError):
        controller.set_command(float("nan"), 0.0, 0.2)

    assert board.commands[-1] == (0.0, 0.0)


def test_ramp_limits_each_control_step():
    board = FakeBoard()
    controller = make_controller(
        board, max_linear_accel=0.35, max_angular_accel=0.70
    )
    controller.set_command(0.18, 0.55, 0.0)
    controller.tick(0.01)

    assert controller.current_vx == pytest.approx(0.0175)
    assert controller.current_vz == pytest.approx(0.035)
