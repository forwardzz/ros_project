#!/usr/bin/env python3
"""Fail-closed V4.0 expansion-board driver for the tracked robot."""

import fcntl
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .motor_board import (
    BcmGpio,
    DirectGpioMotorBoard,
    MOTOR_PAIRS,
    MotorHardwareError,
    Pca9685,
    V40MotorBoard,
)
from .motor_control import MotorControllerCore


class TrackedMotorDriver(Node):
    """Differential-drive controller with explicit hardware enable interlocks."""

    HARD_MAX_PWM = 70.0
    HARD_MAX_LINEAR_SPEED = 0.18
    HARD_MAX_ANGULAR_SPEED = 0.55
    HARD_MAX_LINEAR_ACCEL = 0.35
    HARD_MAX_ANGULAR_ACCEL = 0.70

    def __init__(self):
        super().__init__("tracked_motor_driver")
        self.declare_parameter("motor_pair", "disabled")
        # Raised-track single-channel tests confirmed both logical forward
        # directions match the validated C/left and D/right GPIO interface.
        self.declare_parameter("left_inverted", False)
        self.declare_parameter("right_inverted", False)
        self.declare_parameter("actuation_enabled", False)
        self.declare_parameter("pca_i2c_bus", 1)
        self.declare_parameter("pca_i2c_address", 0x40)
        self.declare_parameter("pca_pwm_frequency_hz", 100.0)
        self.declare_parameter("gpio_pwm_frequency_hz", 100.0)
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
        self.declare_parameter("command_timeout_sec", 0.35)
        self.declare_parameter("watchdog_period_sec", 0.05)
        self.declare_parameter(
            "driver_lock_path", "/tmp/inspection_robot_motor_driver.lock"
        )

        self._driver_lock = None
        self.board = None
        self.controller = None
        self.actuation_enabled = False

        self.max_rpm = self._safe_parameter("max_rpm", 80.0, 1.0, 500.0)
        self.wheel_radius = self._safe_parameter("wheel_radius", 0.025, 0.001, 1.0)
        self.track_width = self._safe_parameter("track_width", 0.155, 0.001, 2.0)
        self.min_breakout_pwm = self._safe_parameter(
            "min_breakout_pwm", 28.0, 0.0, self.HARD_MAX_PWM
        )
        self.max_pwm = self._safe_parameter(
            "max_pwm", 70.0, self.min_breakout_pwm, self.HARD_MAX_PWM
        )
        self.max_linear_speed = self._safe_parameter(
            "max_linear_speed", 0.18, 0.0, self.HARD_MAX_LINEAR_SPEED
        )
        self.max_angular_speed = self._safe_parameter(
            "max_angular_speed", 0.55, 0.0, self.HARD_MAX_ANGULAR_SPEED
        )
        self.max_linear_accel = self._safe_parameter(
            "max_linear_accel", 0.35, 0.0, self.HARD_MAX_LINEAR_ACCEL
        )
        self.max_angular_accel = self._safe_parameter(
            "max_angular_accel", 0.70, 0.0, self.HARD_MAX_ANGULAR_ACCEL
        )
        self.control_period_sec = self._safe_parameter(
            "control_period_sec", 0.05, 0.01, 0.20
        )
        self.command_timeout = self._safe_parameter(
            "command_timeout_sec", 0.35, 0.05, 2.0
        )
        self.watchdog_period = self._safe_parameter(
            "watchdog_period_sec", 0.05, 0.02, 0.20
        )

        requested_enable = bool(self.get_parameter("actuation_enabled").value)
        motor_pair = str(self.get_parameter("motor_pair").value).strip().lower()
        self._acquire_driver_lock()
        if self._driver_lock is None:
            self.get_logger().error("Motor driver lock unavailable; remaining disabled")
        elif requested_enable and motor_pair in ("ab", "cd"):
            self._initialize_hardware(motor_pair)
        elif requested_enable:
            self.get_logger().error(
                "Actuation requested but motor_pair is not 'ab' or 'cd'; remaining disabled"
            )
        else:
            self.get_logger().warn(
                "Motor actuation is disabled by default; outputs will not be initialized"
            )

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_cb, 10
        )
        self.watchdog_timer = self.create_timer(
            self.watchdog_period, self.watchdog_cb
        )
        self.control_timer = self.create_timer(
            self.control_period_sec, self.control_cb
        )

    def _initialize_hardware(self, motor_pair):
        pca = None
        gpio = None
        try:
            if motor_pair == "cd":
                gpio = BcmGpio()
                self.board = DirectGpioMotorBoard(
                    gpio,
                    frequency_hz=self._safe_parameter(
                        "gpio_pwm_frequency_hz", 100.0, 1.0, 5000.0
                    ),
                    left_inverted=bool(
                        self.get_parameter("left_inverted").value
                    ),
                    right_inverted=bool(
                        self.get_parameter("right_inverted").value
                    ),
                )
                backend = "validated GPIO18/22/27 + GPIO23/25/24"
            else:
                channels_to_clear = sorted({
                    pin.number
                    for port in MOTOR_PAIRS[motor_pair]
                    for pin in (port.enable, port.forward, port.reverse)
                    if pin.kind == "pca"
                })
                pca = Pca9685(
                    bus_number=int(self.get_parameter("pca_i2c_bus").value),
                    address=int(self.get_parameter("pca_i2c_address").value),
                    frequency_hz=self._safe_parameter(
                        "pca_pwm_frequency_hz", 100.0, 24.0, 1526.0
                    ),
                    channels_to_clear=channels_to_clear,
                )
                self.board = V40MotorBoard(
                    motor_pair,
                    pca,
                    left_inverted=bool(
                        self.get_parameter("left_inverted").value
                    ),
                    right_inverted=bool(
                        self.get_parameter("right_inverted").value
                    ),
                )
                backend = "PCA9685"
            self.controller = MotorControllerCore(
                self.board,
                max_rpm=self.max_rpm,
                wheel_radius=self.wheel_radius,
                track_width=self.track_width,
                min_breakout_pwm=self.min_breakout_pwm,
                max_pwm=self.max_pwm,
                max_linear_speed=self.max_linear_speed,
                max_angular_speed=self.max_angular_speed,
                max_linear_accel=self.max_linear_accel,
                max_angular_accel=self.max_angular_accel,
                control_period_sec=self.control_period_sec,
                command_timeout_sec=self.command_timeout,
                now_sec=self._now_sec(),
            )
            self.actuation_enabled = True
            self.get_logger().warn(
                "MOTOR ACTUATION ENABLED on pair %s via %s "
                "(left_inverted=%s, right_inverted=%s)"
                % (
                    motor_pair,
                    backend,
                    self.get_parameter("left_inverted").value,
                    self.get_parameter("right_inverted").value,
                )
            )
        except Exception as exc:
            self.get_logger().error(
                "Motor board initialization failed; actuation disabled: %s" % exc
            )
            if gpio is not None:
                try:
                    gpio.close()
                except Exception:
                    pass
            if pca is not None:
                try:
                    pca.close()
                except Exception:
                    pass
            self.board = None
            self.controller = None
            self._release_driver_lock()

    def _safe_parameter(self, name, default, lower, upper):
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            self.get_logger().error(
                "Invalid %s; using safe default %s" % (name, default)
            )
            value = default
        if not math.isfinite(value):
            self.get_logger().error(
                "Non-finite %s; using safe default %s" % (name, default)
            )
            value = default
        return max(lower, min(upper, value))

    def _acquire_driver_lock(self):
        path = str(self.get_parameter("driver_lock_path").value)
        try:
            lock = open(path, "w", encoding="utf-8")
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            lock.write("%d\n" % os.getpid())
            lock.flush()
            self._driver_lock = lock
        except OSError as exc:
            self.get_logger().error(
                "Another motor driver owns the board (%s); remaining disabled: %s"
                % (path, exc)
            )
            try:
                lock.close()
            except (UnboundLocalError, OSError):
                pass

    def _release_driver_lock(self):
        if self._driver_lock is None:
            return
        try:
            fcntl.flock(self._driver_lock.fileno(), fcntl.LOCK_UN)
            self._driver_lock.close()
        except OSError:
            pass
        self._driver_lock = None

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _hardware_call(self, callback):
        if not self.actuation_enabled or self.controller is None:
            return
        try:
            callback()
        except (MotorHardwareError, OSError, ValueError) as exc:
            self.get_logger().fatal(
                "Motor hardware fault; outputs commanded to zero and actuation latched off: %s"
                % exc
            )
            self.actuation_enabled = False
            if self.board is not None:
                try:
                    self.board.stop()
                except Exception:
                    pass

    def cmd_vel_cb(self, msg):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            self.get_logger().error("Invalid /cmd_vel received; stopping motors")
            self._hardware_call(self.controller.stop if self.controller else lambda: None)
            return
        self._hardware_call(
            lambda: self.controller.set_command(linear_x, angular_z, self._now_sec())
        )

    def control_cb(self):
        self._hardware_call(lambda: self.controller.tick(self._now_sec()))

    def watchdog_cb(self):
        self._hardware_call(lambda: self.controller.watchdog(self._now_sec()))

    def destroy_node(self):
        if self.controller is not None:
            try:
                self.controller.stop()
            except Exception:
                pass
        if self.board is not None:
            try:
                self.board.close()
            except Exception:
                pass
        self.actuation_enabled = False
        self._release_driver_lock()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackedMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
