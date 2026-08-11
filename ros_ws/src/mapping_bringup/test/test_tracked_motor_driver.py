"""Hardware-independent tests for the CLB V3.0 direct-GPIO driver."""

import pytest

import mapping_bringup.tracked_motor_driver as driver_module
from mapping_bringup.tracked_motor_driver import (
    DirectGpioMotorBoard,
    LEFT_FORWARD,
    LEFT_PWM,
    LEFT_REVERSE,
    MOTOR_PINS,
    RIGHT_FORWARD,
    RIGHT_PWM,
    RIGHT_REVERSE,
    TrackedMotorDriver,
)


class FakePwm:
    def __init__(self, pin, frequency):
        self.pin = pin
        self.frequency = frequency
        self.events = []

    def start(self, duty):
        self.events.append(("start", duty))

    def ChangeDutyCycle(self, duty):
        self.events.append(("duty", duty))

    def stop(self):
        self.events.append(("stop",))


class FakeGpio:
    BCM = "BCM"
    OUT = "OUT"

    def __init__(self):
        self.events = []
        self.pwms = {}

    def setwarnings(self, enabled):
        self.events.append(("warnings", enabled))

    def setmode(self, mode):
        self.events.append(("mode", mode))

    def setup(self, pin, mode):
        self.events.append(("setup", pin, mode))

    def output(self, pin, value):
        self.events.append(("output", pin, bool(value)))

    def PWM(self, pin, frequency):
        pwm = FakePwm(pin, frequency)
        self.pwms[pin] = pwm
        return pwm

    def cleanup(self, pins):
        self.events.append(("cleanup", tuple(pins)))


def make_board(**kwargs):
    gpio = FakeGpio()
    return gpio, DirectGpioMotorBoard(gpio, **kwargs)


def test_gpio_mapping_matches_the_robot_validated_interface():
    assert (LEFT_PWM, LEFT_FORWARD, LEFT_REVERSE) == (18, 22, 27)
    assert (RIGHT_PWM, RIGHT_FORWARD, RIGHT_REVERSE) == (23, 25, 24)
    assert MOTOR_PINS == (18, 22, 27, 23, 25, 24)


def test_board_initializes_both_pwm_channels_stopped():
    gpio, _board = make_board(frequency_hz=100.0)
    assert set(gpio.pwms) == {18, 23}
    assert gpio.pwms[18].frequency == 100.0
    assert gpio.pwms[23].frequency == 100.0
    assert ("start", 0.0) in gpio.pwms[18].events
    assert ("start", 0.0) in gpio.pwms[23].events


def test_forward_and_reverse_use_gpio_direction_and_pwm_enable():
    gpio, board = make_board()
    gpio.events.clear()
    gpio.pwms[LEFT_PWM].events.clear()

    board.drive_left(49.0)
    assert gpio.events == [
        ("output", LEFT_FORWARD, True),
        ("output", LEFT_REVERSE, False),
    ]
    assert gpio.pwms[LEFT_PWM].events == [("duty", 49.0)]

    gpio.events.clear()
    gpio.pwms[LEFT_PWM].events.clear()
    board.drive_left(-49.0)
    assert gpio.pwms[LEFT_PWM].events == [("duty", 0.0), ("duty", 49.0)]
    assert gpio.events == [
        ("output", LEFT_FORWARD, False),
        ("output", LEFT_REVERSE, False),
        ("output", LEFT_FORWARD, False),
        ("output", LEFT_REVERSE, True),
    ]


def test_each_track_can_be_inverted_independently():
    gpio, board = make_board(left_inverted=True, right_inverted=False)
    gpio.events.clear()
    board.drive_left(40.0)
    board.drive_right(40.0)
    assert ("output", LEFT_FORWARD, False) in gpio.events
    assert ("output", LEFT_REVERSE, True) in gpio.events
    assert ("output", RIGHT_FORWARD, True) in gpio.events
    assert ("output", RIGHT_REVERSE, False) in gpio.events


def test_stop_and_close_clear_outputs_and_only_cleanup_motor_pins():
    gpio, board = make_board()
    board.drive_left(40.0)
    board.drive_right(40.0)
    board.close()
    assert gpio.pwms[LEFT_PWM].events[-2:] == [("duty", 0.0), ("stop",)]
    assert gpio.pwms[RIGHT_PWM].events[-2:] == [("duty", 0.0), ("stop",)]
    assert gpio.events[-1] == ("cleanup", MOTOR_PINS)


def test_rpm_to_duty_applies_breakout_and_sign():
    node = TrackedMotorDriver.__new__(TrackedMotorDriver)
    node.max_rpm = 80.0
    node.min_breakout_pwm = 28.0
    node.max_pwm = 70.0
    assert node._rpm_to_duty(0.5) == 0.0
    assert node._rpm_to_duty(40.0) == pytest.approx(49.0)
    assert node._rpm_to_duty(-40.0) == pytest.approx(-49.0)
    assert node._rpm_to_duty(200.0) == 70.0


def test_set_motor_routes_to_the_correct_logical_track():
    class Board:
        def __init__(self):
            self.calls = []

        def drive_left(self, duty):
            self.calls.append(("left", duty))

        def drive_right(self, duty):
            self.calls.append(("right", duty))

    node = TrackedMotorDriver.__new__(TrackedMotorDriver)
    node.board = Board()
    node._rpm_to_duty = lambda rpm: rpm + 1.0
    node.set_motor(True, 10.0)
    node.set_motor(False, -10.0)
    assert node.board.calls == [("left", 11.0), ("right", -9.0)]


def test_motion_helpers_clamp_and_ramp():
    assert TrackedMotorDriver._clamp(2.0, -1.0, 1.0) == 1.0
    assert TrackedMotorDriver._step_toward(0.0, 1.0, 0.2) == 0.2
    assert TrackedMotorDriver._step_toward(0.0, -1.0, 0.2) == -0.2


def test_watchdog_immediately_stops_all_motor_outputs():
    class Stamp:
        def __init__(self, seconds):
            self.nanoseconds = int(seconds * 1e9)

        def __sub__(self, other):
            return Stamp((self.nanoseconds - other.nanoseconds) / 1e9)

    class Clock:
        def now(self):
            return Stamp(1.0)

    class Logger:
        def warning(self, _message):
            pass

    node = TrackedMotorDriver.__new__(TrackedMotorDriver)
    node.get_clock = lambda: Clock()
    node.get_logger = lambda: Logger()
    node.last_cmd_time = Stamp(0.0)
    node.watchdog_timeout_sec = 0.5
    node._watchdog_stopped = False
    node.target_vx = node.target_vz = 0.1
    node.current_vx = node.current_vz = 0.1
    stops = []
    node.stop_all = lambda: stops.append(True)

    node.watchdog_cb()
    assert stops == [True]
    assert node._watchdog_stopped is True
    assert (node.target_vx, node.target_vz) == (0.0, 0.0)
    assert (node.current_vx, node.current_vz) == (0.0, 0.0)
