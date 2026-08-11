#!/usr/bin/env python3
"""Direct-GPIO motor driver for the robot's CLB V3.0 expansion board.

The board's PCA9685 drives its three-pin servo headers.  The two-pin motor
outputs are controlled by two GPIO PWM enables and four GPIO direction pins.
Each logical side is wired to two physical motor sockets on the board.
"""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


LEFT_PWM = 18
LEFT_FORWARD = 22
LEFT_REVERSE = 27
RIGHT_PWM = 23
RIGHT_FORWARD = 25
RIGHT_REVERSE = 24
MOTOR_PINS = (
    LEFT_PWM,
    LEFT_FORWARD,
    LEFT_REVERSE,
    RIGHT_PWM,
    RIGHT_FORWARD,
    RIGHT_REVERSE,
)


class DirectGpioMotorBoard:
    """Fail-closed adapter for the GPIO interface validated on this robot."""

    def __init__(
        self,
        gpio,
        frequency_hz=100.0,
        left_inverted=False,
        right_inverted=False,
    ):
        if gpio is None:
            raise RuntimeError(
                "RPi.GPIO is unavailable; install the python3-rpi-gpio package"
            )
        frequency_hz = float(frequency_hz)
        if not math.isfinite(frequency_hz) or not 1.0 <= frequency_hz <= 5000.0:
            raise ValueError("gpio_pwm_frequency_hz must be between 1 and 5000")

        self.gpio = gpio
        self.left_inverted = bool(left_inverted)
        self.right_inverted = bool(right_inverted)
        self.left_pwm = None
        self.right_pwm = None
        self._left_direction = None
        self._right_direction = None
        self.closed = False

        self.gpio.setwarnings(False)
        self.gpio.setmode(self.gpio.BCM)
        for pin in MOTOR_PINS:
            self.gpio.setup(pin, self.gpio.OUT)
            self.gpio.output(pin, False)

        self.left_pwm = self.gpio.PWM(LEFT_PWM, frequency_hz)
        self.right_pwm = self.gpio.PWM(RIGHT_PWM, frequency_hz)
        self.left_pwm.start(0.0)
        self.right_pwm.start(0.0)
        self.stop()

    def _drive_one(
        self,
        pwm,
        forward_pin,
        reverse_pin,
        signed_duty,
        previous_direction,
    ):
        signed_duty = float(signed_duty)
        if not math.isfinite(signed_duty):
            raise ValueError("motor duty must be finite")
        signed_duty = max(-100.0, min(100.0, signed_duty))

        if abs(signed_duty) <= 1e-9:
            pwm.ChangeDutyCycle(0.0)
            self.gpio.output(forward_pin, False)
            self.gpio.output(reverse_pin, False)
            return None

        forward = signed_duty > 0.0
        if previous_direction is not None and previous_direction != forward:
            # Remove the PWM enable before reversing the H bridge.
            pwm.ChangeDutyCycle(0.0)
            self.gpio.output(forward_pin, False)
            self.gpio.output(reverse_pin, False)
            time.sleep(0.001)

        self.gpio.output(forward_pin, forward)
        self.gpio.output(reverse_pin, not forward)
        pwm.ChangeDutyCycle(abs(signed_duty))
        return forward

    def drive_left(self, duty):
        if self.closed:
            raise RuntimeError("motor board is closed")
        if self.left_inverted:
            duty = -float(duty)
        self._left_direction = self._drive_one(
            self.left_pwm,
            LEFT_FORWARD,
            LEFT_REVERSE,
            duty,
            self._left_direction,
        )

    def drive_right(self, duty):
        if self.closed:
            raise RuntimeError("motor board is closed")
        if self.right_inverted:
            duty = -float(duty)
        self._right_direction = self._drive_one(
            self.right_pwm,
            RIGHT_FORWARD,
            RIGHT_REVERSE,
            duty,
            self._right_direction,
        )

    def stop(self):
        first_error = None
        for pwm in (self.left_pwm, self.right_pwm):
            if pwm is not None:
                try:
                    pwm.ChangeDutyCycle(0.0)
                except Exception as exc:
                    first_error = first_error or exc
        for pin in (LEFT_FORWARD, LEFT_REVERSE, RIGHT_FORWARD, RIGHT_REVERSE):
            try:
                self.gpio.output(pin, False)
            except Exception as exc:
                first_error = first_error or exc
        self._left_direction = None
        self._right_direction = None
        if first_error is not None:
            raise RuntimeError("failed to clear motor outputs") from first_error

    def close(self):
        if self.closed:
            return
        try:
            self.stop()
        finally:
            self.closed = True
            for pwm in (self.left_pwm, self.right_pwm):
                if pwm is not None:
                    try:
                        pwm.stop()
                    except Exception:
                        pass
            try:
                self.gpio.cleanup(MOTOR_PINS)
            except Exception:
                pass


class TrackedMotorDriver(Node):
    def __init__(self):
        super().__init__("tracked_motor_driver")

        self.declare_parameter("max_rpm", 80.0)
        self.declare_parameter("wheel_radius", 0.025)
        self.declare_parameter("track_width", 0.155)
        self.declare_parameter("min_breakout_pwm", 28.0)
        self.declare_parameter("max_pwm", 70.0)
        self.declare_parameter("max_linear_speed", 0.18)
        self.declare_parameter("max_angular_speed", 0.55)
        self.declare_parameter("max_linear_accel", 0.35)
        self.declare_parameter("max_angular_accel", 0.70)
        self.declare_parameter("control_period_sec", 0.05)
        self.declare_parameter("watchdog_timeout_sec", 0.5)
        self.declare_parameter("gpio_pwm_frequency_hz", 100.0)
        self.declare_parameter("left_inverted", False)
        self.declare_parameter("right_inverted", False)

        self.max_rpm = self._float_param("max_rpm")
        self.wheel_radius = self._float_param("wheel_radius")
        self.track_width = self._float_param("track_width")
        self.min_breakout_pwm = self._float_param("min_breakout_pwm")
        self.max_pwm = self._float_param("max_pwm")
        self.max_linear_speed = self._float_param("max_linear_speed")
        self.max_angular_speed = self._float_param("max_angular_speed")
        self.max_linear_accel = self._float_param("max_linear_accel")
        self.max_angular_accel = self._float_param("max_angular_accel")
        self.control_period_sec = self._float_param("control_period_sec")
        self.watchdog_timeout_sec = self._float_param("watchdog_timeout_sec")
        self.gpio_pwm_frequency_hz = self._float_param("gpio_pwm_frequency_hz")
        self._validate_parameters()

        self.board = None
        try:
            self.board = DirectGpioMotorBoard(
                GPIO,
                frequency_hz=self.gpio_pwm_frequency_hz,
                left_inverted=bool(self.get_parameter("left_inverted").value),
                right_inverted=bool(self.get_parameter("right_inverted").value),
            )
        except Exception as exc:
            self.get_logger().fatal(f"Motor hardware initialization failed: {exc}")
            raise

        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.target_vx = 0.0
        self.target_vz = 0.0
        self.current_vx = 0.0
        self.current_vz = 0.0
        self._watchdog_stopped = False
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_cb, 10
        )
        self.control_timer = self.create_timer(
            self.control_period_sec, self.control_cb
        )
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_cb)
        self.get_logger().info(
            "Motor driver ready: CLB V3.0 direct GPIO; "
            "left PWM/IN1/IN2=18/22/27, right=23/25/24"
        )

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    def _validate_parameters(self):
        for name, value in (
            ("max_rpm", self.max_rpm),
            ("wheel_radius", self.wheel_radius),
            ("track_width", self.track_width),
            ("control_period_sec", self.control_period_sec),
            ("watchdog_timeout_sec", self.watchdog_timeout_sec),
            ("gpio_pwm_frequency_hz", self.gpio_pwm_frequency_hz),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be greater than zero")
        self.max_pwm = max(0.0, min(self.max_pwm, 100.0))
        self.min_breakout_pwm = max(
            0.0, min(self.min_breakout_pwm, self.max_pwm)
        )

    def _rpm_to_duty(self, target_rpm):
        target_rpm = float(target_rpm)
        if abs(target_rpm) <= 0.5:
            return 0.0
        abs_rpm = min(abs(target_rpm), self.max_rpm)
        duty = self.min_breakout_pwm + (
            (self.max_pwm - self.min_breakout_pwm) * abs_rpm / self.max_rpm
        )
        return math.copysign(duty, target_rpm)

    def set_motor(self, is_left, target_rpm):
        duty = self._rpm_to_duty(target_rpm)
        if is_left:
            self.board.drive_left(duty)
        else:
            self.board.drive_right(duty)

    def stop_all(self):
        if self.board is not None:
            self.board.stop()
        self.left_rpm = 0.0
        self.right_rpm = 0.0

    def cmd_vel_cb(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self._watchdog_stopped = False
        self.target_vx = self._clamp(
            msg.linear.x, -self.max_linear_speed, self.max_linear_speed
        )
        self.target_vz = self._clamp(
            msg.angular.z, -self.max_angular_speed, self.max_angular_speed
        )

    def control_cb(self):
        if self._watchdog_stopped:
            return
        self.current_vx = self._step_toward(
            self.current_vx,
            self.target_vx,
            self.max_linear_accel * self.control_period_sec,
        )
        self.current_vz = self._step_toward(
            self.current_vz,
            self.target_vz,
            self.max_angular_accel * self.control_period_sec,
        )
        left_rad_s = (
            self.current_vx - self.current_vz * self.track_width / 2.0
        ) / self.wheel_radius
        right_rad_s = (
            self.current_vx + self.current_vz * self.track_width / 2.0
        ) / self.wheel_radius
        self.left_rpm = self._clamp(
            left_rad_s * 60.0 / (2.0 * math.pi),
            -self.max_rpm,
            self.max_rpm,
        )
        self.right_rpm = self._clamp(
            right_rad_s * 60.0 / (2.0 * math.pi),
            -self.max_rpm,
            self.max_rpm,
        )
        try:
            self.set_motor(True, self.left_rpm)
            self.set_motor(False, self.right_rpm)
        except Exception as exc:
            try:
                self.stop_all()
            finally:
                self._watchdog_stopped = True
            self.get_logger().fatal(f"Motor output failed; motors stopped: {exc}")
            raise

    def watchdog_cb(self):
        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout_sec and not self._watchdog_stopped:
            self.target_vx = 0.0
            self.target_vz = 0.0
            self.current_vx = 0.0
            self.current_vz = 0.0
            self.stop_all()
            self._watchdog_stopped = True
            self.get_logger().warning(
                f"/cmd_vel timeout ({elapsed:.2f}s): all motors stopped"
            )

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, float(value)))

    @staticmethod
    def _step_toward(current, target, max_step):
        if target > current:
            return min(target, current + max_step)
        if target < current:
            return max(target, current - max_step)
        return current

    def _cleanup_hardware(self):
        if self.board is not None:
            try:
                self.board.close()
            except Exception:
                pass

    def destroy_node(self):
        self._cleanup_hardware()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackedMotorDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
