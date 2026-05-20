#!/usr/bin/env python3
"""
Tracked vehicle motor driver using RPi.GPIO direct drive.
Differential drive with TT Motor 130 (1:48) via L298N/TB6612.
Each motor has independent direction pins — turning supported.

Dead-zone handling:
  - PWM < min_breakout_pwm → motor = 0 (completely off)
  - PWM between [min_breakout_pwm, 100] → linear map from breakout RPM to max RPM
  This ensures Nav2 small velocities still get enough PWM to overcome static friction.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class TrackedMotorDriver(Node):
    def __init__(self):
        super().__init__("tracked_motor_driver")

        # Physical params
        self.max_rpm = 80.0
        self.wheel_radius = 0.025      # meters
        self.track_width = 0.155       # meters
        self.min_breakout_pwm = 28.0   # minimum PWM % that overcomes static friction

        # BCM GPIO pins
        self.PWMA, self.AIN1, self.AIN2 = 18, 22, 27   # left motor
        self.PWMB, self.BIN1, self.BIN2 = 23, 25, 24   # right motor

        if HAS_GPIO:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for pin in [self.PWMA, self.AIN1, self.AIN2,
                        self.PWMB, self.BIN1, self.BIN2]:
                GPIO.setup(pin, GPIO.OUT)

            self.L_Motor = GPIO.PWM(self.PWMA, 100)
            self.R_Motor = GPIO.PWM(self.PWMB, 100)
            self.L_Motor.start(0)
            self.R_Motor.start(0)
            self.get_logger().info(
                f"GPIO motor pins initialized, breakout pwm={self.min_breakout_pwm}%")
        else:
            self.get_logger().error("RPi.GPIO not available")

        # State tracking for odometry
        self.left_rpm = 0.0
        self.right_rpm = 0.0

        # ROS interface
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_cb)
        self.get_logger().info("Motor driver ready, waiting for /cmd_vel ...")

    def set_motor(self, is_left, target_rpm):
        """Control single motor with dead-zone compensation.

        RPM → Duty Cycle mapping:
          - |rpm| close to 0 → 0% (off)
          - |rpm| > 0 → mapped to [min_breakout_pwm, 100]% linearly
        """
        if not HAS_GPIO:
            return

        if is_left:
            in1_pin, in2_pin, pwm_obj = self.AIN1, self.AIN2, self.L_Motor
        else:
            in1_pin, in2_pin, pwm_obj = self.BIN1, self.BIN2, self.R_Motor

        # Determine min RPM that the breakout PWM would produce
        min_breakout_rpm = self.max_rpm * (self.min_breakout_pwm / 100.0)

        abs_rpm = abs(target_rpm)

        if abs_rpm <= 0.5:  # effectively zero
            pwm_obj.ChangeDutyCycle(0)
            GPIO.output(in1_pin, False)
            GPIO.output(in2_pin, False)
            return

        # Cap at max
        abs_rpm = min(abs_rpm, self.max_rpm)

        # Linear map: [0, max_rpm] → [min_breakout_pwm, 100]
        duty = self.min_breakout_pwm + (100.0 - self.min_breakout_pwm) * (abs_rpm / self.max_rpm)

        # Direction
        if target_rpm > 0:
            GPIO.output(in1_pin, True)
            GPIO.output(in2_pin, False)
        else:
            GPIO.output(in1_pin, False)
            GPIO.output(in2_pin, True)

        pwm_obj.ChangeDutyCycle(duty)

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        vx = msg.linear.x
        vz = msg.angular.z

        left_rad_s = (vx - vz * self.track_width / 2.0) / self.wheel_radius
        right_rad_s = (vx + vz * self.track_width / 2.0) / self.wheel_radius

        left_rpm = left_rad_s * 60.0 / (2.0 * math.pi)
        right_rpm = right_rad_s * 60.0 / (2.0 * math.pi)

        self.left_rpm = left_rpm
        self.right_rpm = right_rpm

        self.set_motor(True, left_rpm)
        self.set_motor(False, right_rpm)

    def watchdog_cb(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            self.left_rpm = 0.0
            self.right_rpm = 0.0
            self.set_motor(True, 0)
            self.set_motor(False, 0)

    def destroy_node(self):
        self.set_motor(True, 0)
        self.set_motor(False, 0)
        if HAS_GPIO:
            GPIO.cleanup()
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
