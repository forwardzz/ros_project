"""Hardware-independent differential drive and watchdog logic."""

import math


class MotorControllerCore:
    """Convert bounded Twist targets to signed motor duty cycles."""

    def __init__(
        self,
        board,
        *,
        max_rpm,
        wheel_radius,
        track_width,
        min_breakout_pwm,
        max_pwm,
        max_linear_speed,
        max_angular_speed,
        max_linear_accel,
        max_angular_accel,
        control_period_sec,
        command_timeout_sec,
        now_sec,
    ):
        self.board = board
        self.max_rpm = max_rpm
        self.wheel_radius = wheel_radius
        self.track_width = track_width
        self.min_breakout_pwm = min_breakout_pwm
        self.max_pwm = max_pwm
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        self.control_period_sec = control_period_sec
        self.command_timeout_sec = command_timeout_sec
        self.last_command_sec = float(now_sec)
        self.target_vx = self.target_vz = 0.0
        self.current_vx = self.current_vz = 0.0
        self.left_rpm = self.right_rpm = 0.0

    def set_command(self, linear_x, angular_z, now_sec):
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            self.stop()
            raise ValueError("velocity command must be finite")
        self.last_command_sec = float(now_sec)
        self.target_vx = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        self.target_vz = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

    def is_stale(self, now_sec):
        return float(now_sec) - self.last_command_sec > self.command_timeout_sec

    def watchdog(self, now_sec):
        if self.is_stale(now_sec):
            self.stop()
            return True
        return False

    def tick(self, now_sec):
        if self.is_stale(now_sec):
            self.stop()
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
        self.left_rpm = left_rad_s * 60.0 / (2.0 * math.pi)
        self.right_rpm = right_rad_s * 60.0 / (2.0 * math.pi)
        self.board.drive(self._rpm_to_duty(self.left_rpm), self._rpm_to_duty(self.right_rpm))

    def _rpm_to_duty(self, rpm):
        if not math.isfinite(rpm) or abs(rpm) <= 0.5:
            return 0.0
        magnitude = min(abs(rpm), self.max_rpm)
        duty = self.min_breakout_pwm + (
            self.max_pwm - self.min_breakout_pwm
        ) * magnitude / self.max_rpm
        return math.copysign(min(self.max_pwm, duty), rpm)

    def stop(self):
        self.target_vx = self.target_vz = 0.0
        self.current_vx = self.current_vz = 0.0
        self.left_rpm = self.right_rpm = 0.0
        self.board.stop()

    @staticmethod
    def _step_toward(current, target, max_step):
        if target > current:
            return min(target, current + max_step)
        if target < current:
            return max(target, current - max_step)
        return current
