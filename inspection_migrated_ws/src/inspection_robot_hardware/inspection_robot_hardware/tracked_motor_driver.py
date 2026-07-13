#!/usr/bin/env python3
"""Fail-closed GPIO driver for the tracked inspection vehicle.

The driver is intentionally the final hardware boundary: it accepts only a
bounded ``geometry_msgs/Twist`` command and immediately de-energizes both
motors when the command stream is invalid or stale.  It provides no odometry;
all RPM values below are open-loop estimates.
"""

import fcntl
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class TrackedMotorDriver(Node):
    """Direct differential-drive GPIO controller with a watchdog interlock."""

    # Do not make a physical vehicle faster by modifying a launch parameter.
    HARD_MAX_PWM = 70.0
    HARD_MAX_LINEAR_SPEED = 0.18       # m/s
    HARD_MAX_ANGULAR_SPEED = 0.55      # rad/s
    HARD_MAX_LINEAR_ACCEL = 0.35       # m/s²
    HARD_MAX_ANGULAR_ACCEL = 0.70      # rad/s²

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
        self.declare_parameter("command_timeout_sec", 0.35)
        self.declare_parameter("watchdog_period_sec", 0.05)
        self.declare_parameter(
            "driver_lock_path", "/tmp/inspection_robot_motor_driver.lock"
        )

        self._driver_lock = None
        self.gpio_initialized = False
        self.actuation_enabled = False

        self.max_rpm = self._safe_parameter("max_rpm", 80.0, 1.0, 500.0)
        self.wheel_radius = self._safe_parameter("wheel_radius", 0.025, 0.001, 1.0)
        self.track_width = self._safe_parameter("track_width", 0.155, 0.001, 2.0)
        self.min_breakout_pwm = self._safe_parameter("min_breakout_pwm", 28.0, 0.0, self.HARD_MAX_PWM)
        self.max_pwm = self._safe_parameter("max_pwm", 70.0, self.min_breakout_pwm, self.HARD_MAX_PWM)
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
        self.control_period_sec = self._safe_parameter("control_period_sec", 0.05, 0.01, 0.20)
        self.command_timeout = self._safe_parameter("command_timeout_sec", 0.35, 0.05, 2.0)
        self.watchdog_period = self._safe_parameter("watchdog_period_sec", 0.05, 0.02, 0.20)

        # BCM GPIO pins. Positive rpm is the assumed physical forward direction;
        # it must be confirmed with wheels suspended before floor testing.
        self.PWMA, self.AIN1, self.AIN2 = 18, 22, 27   # left motor
        self.PWMB, self.BIN1, self.BIN2 = 23, 25, 24   # right motor

        self._acquire_driver_lock()
        if self._driver_lock is not None and HAS_GPIO:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                for pin in [self.PWMA, self.AIN1, self.AIN2,
                            self.PWMB, self.BIN1, self.BIN2]:
                    GPIO.setup(pin, GPIO.OUT)
                self.L_Motor = GPIO.PWM(self.PWMA, 100)
                self.R_Motor = GPIO.PWM(self.PWMB, 100)
                self.L_Motor.start(0)
                self.R_Motor.start(0)
                self.gpio_initialized = True
                self.actuation_enabled = True
                self.get_logger().info(
                    "GPIO motor pins initialized (hard limits: %.2f m/s, %.2f rad/s, %.0f%% PWM)"
                    % (self.max_linear_speed, self.max_angular_speed, self.max_pwm)
                )
            except Exception as exc:
                self.get_logger().error("GPIO initialization failed; actuation disabled: %s" % exc)
                self._release_driver_lock()
        elif not HAS_GPIO:
            self.get_logger().error("RPi.GPIO is unavailable; actuation disabled")

        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.target_vx = 0.0
        self.target_vz = 0.0
        self.current_vx = 0.0
        self.current_vz = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.watchdog_timer = self.create_timer(self.watchdog_period, self.watchdog_cb)
        self.control_timer = self.create_timer(self.control_period_sec, self.control_cb)
        if self.actuation_enabled:
            self.get_logger().info("Motor driver ready; stale or invalid /cmd_vel stops both motors")
        else:
            self.get_logger().error("Motor driver remains non-actuating (fail closed)")

    def _safe_parameter(self, name, default, lower, upper):
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            self.get_logger().error("Invalid %s; using safe default %s" % (name, default))
            value = default
        if not math.isfinite(value):
            self.get_logger().error("Non-finite %s; using safe default %s" % (name, default))
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
                "Another motor driver owns GPIO (%s); this instance is non-actuating: %s"
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

    def set_motor(self, is_left, target_rpm):
        """Control one motor; any invalid request de-energizes that motor."""
        if not self.actuation_enabled or not self.gpio_initialized:
            return
        if not math.isfinite(target_rpm):
            target_rpm = 0.0

        if is_left:
            in1_pin, in2_pin, pwm_obj = self.AIN1, self.AIN2, self.L_Motor
        else:
            in1_pin, in2_pin, pwm_obj = self.BIN1, self.BIN2, self.R_Motor

        abs_rpm = abs(target_rpm)
        if abs_rpm <= 0.5:
            pwm_obj.ChangeDutyCycle(0)
            GPIO.output(in1_pin, False)
            GPIO.output(in2_pin, False)
            return

        abs_rpm = min(abs_rpm, self.max_rpm)
        duty = self.min_breakout_pwm + (self.max_pwm - self.min_breakout_pwm) * (abs_rpm / self.max_rpm)
        if target_rpm > 0:
            GPIO.output(in1_pin, True)
            GPIO.output(in2_pin, False)
        else:
            GPIO.output(in1_pin, False)
            GPIO.output(in2_pin, True)
        pwm_obj.ChangeDutyCycle(duty)

    def _force_stop(self):
        self.target_vx = self.target_vz = 0.0
        self.current_vx = self.current_vz = 0.0
        self.left_rpm = self.right_rpm = 0.0
        self.set_motor(True, 0.0)
        self.set_motor(False, 0.0)

    def cmd_vel_cb(self, msg):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            self.get_logger().error("Invalid /cmd_vel received; stopping motors")
            self._force_stop()
            return
        self.last_cmd_time = self.get_clock().now()
        self.target_vx = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        self.target_vz = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

    def control_cb(self):
        if not self.actuation_enabled:
            return
        if self._command_is_stale():
            self._force_stop()
            return
        self.current_vx = self._step_toward(
            self.current_vx, self.target_vx, self.max_linear_accel * self.control_period_sec
        )
        self.current_vz = self._step_toward(
            self.current_vz, self.target_vz, self.max_angular_accel * self.control_period_sec
        )
        left_rad_s = (self.current_vx - self.current_vz * self.track_width / 2.0) / self.wheel_radius
        right_rad_s = (self.current_vx + self.current_vz * self.track_width / 2.0) / self.wheel_radius
        self.left_rpm = left_rad_s * 60.0 / (2.0 * math.pi)
        self.right_rpm = right_rad_s * 60.0 / (2.0 * math.pi)
        self.set_motor(True, self.left_rpm)
        self.set_motor(False, self.right_rpm)

    def _command_is_stale(self):
        return (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9 > self.command_timeout

    def watchdog_cb(self):
        if self._command_is_stale():
            self._force_stop()

    @staticmethod
    def _step_toward(current, target, max_step):
        if target > current:
            return min(target, current + max_step)
        if target < current:
            return max(target, current - max_step)
        return current

    def destroy_node(self):
        self._force_stop()
        if self.gpio_initialized:
            try:
                GPIO.cleanup()
            except Exception:
                pass
            self.gpio_initialized = False
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
