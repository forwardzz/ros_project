#!/usr/bin/env python3
"""TT Motor 130 (1:48) differential drive controller for L298N/TB6612."""

import math
from rclpy.node import Node

try:
    from gpiozero import PWMOutputDevice, DigitalOutputDevice
    from gpiozero.pins.lgpio import LGPIOFactory
    from gpiozero import Device as GPIODevice
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class MotorDriver:
    """Differential drive motor controller."""

    LEFT_PWM = 12
    LEFT_IN1 = 5
    LEFT_IN2 = 6
    RIGHT_PWM = 13
    RIGHT_IN1 = 17
    RIGHT_IN2 = 27

    MAX_RPM = 120.0
    WHEEL_RADIUS = 0.0325
    WHEEL_SEPARATION = 0.14
    PWM_FREQ = 1000

    def __init__(self, node: Node):
        self.node = node
        self._init_gpio()

    def _init_gpio(self):
        if not HAS_GPIO:
            self.node.get_logger().warn('gpiozero not available')
            self.left_pwm = None
            self.right_pwm = None
            return

        try:
            GPIODevice.pin_factory = LGPIOFactory()
        except Exception:
            pass

        try:
            self.left_in1 = DigitalOutputDevice(self.LEFT_IN1)
            self.left_in2 = DigitalOutputDevice(self.LEFT_IN2)
            self.right_in1 = DigitalOutputDevice(self.RIGHT_IN1)
            self.right_in2 = DigitalOutputDevice(self.RIGHT_IN2)
            self.left_pwm = PWMOutputDevice(self.LEFT_PWM, frequency=self.PWM_FREQ)
            self.right_pwm = PWMOutputDevice(self.RIGHT_PWM, frequency=self.PWM_FREQ)
            self.node.get_logger().info('Motor GPIO initialized')
        except Exception as e:
            self.node.get_logger().warn(f'GPIO init failed: {e}')
            self.left_pwm = None
            self.right_pwm = None

    def set_speeds(self, left_rpm: float, right_rpm: float):
        if self.left_pwm is None:
            return
        left_rpm = max(-self.MAX_RPM, min(self.MAX_RPM, left_rpm))
        right_rpm = max(-self.MAX_RPM, min(self.MAX_RPM, right_rpm))
        left_duty = abs(left_rpm) / self.MAX_RPM
        right_duty = abs(right_rpm) / self.MAX_RPM
        self._set_one(self.left_in1, self.left_in2, self.left_pwm,
                      left_rpm >= 0, left_duty)
        self._set_one(self.right_in1, self.right_in2, self.right_pwm,
                      right_rpm >= 0, right_duty)

    def _set_one(self, in1, in2, pwm, forward: bool, duty: float):
        duty = max(0.0, min(1.0, duty))
        if forward:
            in1.on()
            in2.off()
        else:
            in1.off()
            in2.on()
        pwm.value = duty

    def cmd_vel_to_rpm(self, vx: float, vz: float):
        sep = self.WHEEL_SEPARATION
        rad = self.WHEEL_RADIUS
        left_rad_s = (vx - vz * sep / 2.0) / rad
        right_rad_s = (vx + vz * sep / 2.0) / rad
        left_rpm = left_rad_s * 60.0 / (2.0 * math.pi)
        right_rpm = right_rad_s * 60.0 / (2.0 * math.pi)
        return left_rpm, right_rpm

    def stop(self):
        if self.left_pwm is not None:
            self.left_pwm.value = 0.0
            self.right_pwm.value = 0.0

    def shutdown(self):
        self.stop()
