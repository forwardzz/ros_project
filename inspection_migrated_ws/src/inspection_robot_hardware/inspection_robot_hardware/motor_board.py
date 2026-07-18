"""Low-level adapters for the robot motor expansion board."""

from dataclasses import dataclass
import math
import time


class MotorHardwareError(RuntimeError):
    """Raised after an output failure has latched the adapter off."""


class Pca9685:
    """Minimal PCA9685 driver used only for the motor output channels."""

    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(
        self,
        bus_number=1,
        address=0x40,
        frequency_hz=100.0,
        bus_factory=None,
        channels_to_clear=(),
    ):
        if bus_factory is None:
            try:
                from smbus2 import SMBus
            except ImportError:
                from smbus import SMBus
            bus_factory = SMBus
        self.address = int(address)
        self.bus = bus_factory(int(bus_number))
        for channel in channels_to_clear:
            self.set_duty_cycle(channel, 0.0)
        self.set_frequency(frequency_hz)

    def set_frequency(self, frequency_hz):
        frequency_hz = float(frequency_hz)
        if not math.isfinite(frequency_hz) or not 24.0 <= frequency_hz <= 1526.0:
            raise ValueError("PCA9685 frequency must be between 24 and 1526 Hz")
        prescale = int(round(25_000_000.0 / (4096.0 * frequency_hz)) - 1)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        self.bus.write_byte_data(self.address, self.MODE1, (old_mode & 0x7F) | 0x10)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0xA0)

    def set_duty_cycle(self, channel, duty_percent):
        if not 0 <= int(channel) <= 15:
            raise ValueError("PCA9685 channel must be in [0, 15]")
        duty_percent = max(0.0, min(100.0, float(duty_percent)))
        register = self.LED0_ON_L + 4 * int(channel)
        if duty_percent <= 0.0:
            values = [0x00, 0x00, 0x00, 0x10]  # full off
        elif duty_percent >= 100.0:
            values = [0x00, 0x10, 0x00, 0x00]  # full on
        else:
            off_count = min(4095, max(1, int(round(4095.0 * duty_percent / 100.0))))
            values = [0x00, 0x00, off_count & 0xFF, (off_count >> 8) & 0x0F]
        self.bus.write_i2c_block_data(self.address, register, values)

    def close(self):
        self.bus.close()


class BcmGpio:
    """Small RPi.GPIO wrapper so direct motor output remains testable."""

    def __init__(self, module=None):
        if module is None:
            import RPi.GPIO as module
        self.module = module
        module.setwarnings(False)
        module.setmode(module.BCM)
        self.configured_pins = set()

    def setup_output(self, pin):
        self.module.setup(int(pin), self.module.OUT, initial=self.module.LOW)
        self.configured_pins.add(int(pin))

    def write(self, pin, enabled):
        self.module.output(int(pin), self.module.HIGH if enabled else self.module.LOW)

    def create_pwm(self, pin, frequency_hz):
        pwm = self.module.PWM(int(pin), float(frequency_hz))
        pwm.start(0.0)
        return pwm

    def close(self):
        for pin in self.configured_pins:
            try:
                self.module.output(pin, self.module.LOW)
            except Exception:
                pass
        if self.configured_pins:
            self.module.cleanup(sorted(self.configured_pins))
        self.configured_pins.clear()


@dataclass(frozen=True)
class OutputPin:
    kind: str
    number: int


@dataclass(frozen=True)
class MotorPort:
    enable: OutputPin
    forward: OutputPin
    reverse: OutputPin


def pca_pin(channel):
    return OutputPin("pca", channel)


def gpio_pin(pin):
    return OutputPin("gpio", pin)


MOTOR_PAIRS = {
    "ab": (
        MotorPort(pca_pin(0), pca_pin(2), pca_pin(1)),
        MotorPort(pca_pin(5), pca_pin(3), pca_pin(4)),
    ),
}


class V40MotorBoard:
    """Drive one selected motor pair and fail closed after any output error."""

    def __init__(self, motor_pair, pca, gpio=None, left_inverted=False, right_inverted=False):
        if motor_pair not in MOTOR_PAIRS:
            raise ValueError("PCA motor_pair must be 'ab'")
        self.motor_pair = motor_pair
        self.left_port, self.right_port = MOTOR_PAIRS[motor_pair]
        self.pca = pca
        self.gpio = gpio
        self.left_inverted = bool(left_inverted)
        self.right_inverted = bool(right_inverted)
        self.faulted = False
        self.closed = False
        gpio_pins = {
            pin.number
            for port in (self.left_port, self.right_port)
            for pin in (port.enable, port.forward, port.reverse)
            if pin.kind == "gpio"
        }
        if gpio_pins and gpio is None:
            raise ValueError("selected motor pair requires a GPIO backend")
        for pin in sorted(gpio_pins):
            gpio.setup_output(pin)
        self.stop()

    def _write(self, pin, duty_percent):
        if pin.kind == "pca":
            self.pca.set_duty_cycle(pin.number, duty_percent)
        else:
            self.gpio.write(pin.number, duty_percent >= 50.0)

    def _stop_port(self, port):
        # Disable first, then clear both direction inputs.
        self._write(port.enable, 0.0)
        self._write(port.forward, 0.0)
        self._write(port.reverse, 0.0)

    def _best_effort_stop(self):
        first_error = None
        for port in (self.left_port, self.right_port):
            for pin in (port.enable, port.forward, port.reverse):
                try:
                    self._write(pin, 0.0)
                except Exception as exc:
                    if first_error is None:
                        first_error = exc
        return first_error

    def stop(self):
        error = self._best_effort_stop()
        if error is not None:
            self.faulted = True
            raise MotorHardwareError("failed to clear motor outputs") from error

    def drive(self, left_duty, right_duty):
        if self.closed or self.faulted:
            raise MotorHardwareError("motor adapter is closed or fault-latched")
        try:
            self._drive_port(
                self.left_port, -float(left_duty) if self.left_inverted else float(left_duty)
            )
            self._drive_port(
                self.right_port, -float(right_duty) if self.right_inverted else float(right_duty)
            )
        except Exception as exc:
            self.faulted = True
            self._best_effort_stop()
            raise MotorHardwareError("motor output failed; adapter latched off") from exc

    def _drive_port(self, port, signed_duty):
        if not math.isfinite(signed_duty):
            raise ValueError("motor duty must be finite")
        signed_duty = max(-100.0, min(100.0, signed_duty))
        # Always remove enable before changing direction.
        self._write(port.enable, 0.0)
        if abs(signed_duty) <= 1e-9:
            self._write(port.forward, 0.0)
            self._write(port.reverse, 0.0)
            return
        self._write(port.forward, 100.0 if signed_duty > 0.0 else 0.0)
        self._write(port.reverse, 0.0 if signed_duty > 0.0 else 100.0)
        self._write(port.enable, abs(signed_duty))

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._best_effort_stop()
        try:
            if self.gpio is not None:
                self.gpio.close()
        finally:
            self.pca.close()


class DirectGpioMotorBoard:
    """Use the GPIO interface validated by the previous real-robot project."""

    LEFT_PWM = 18
    LEFT_FORWARD = 22
    LEFT_REVERSE = 27
    RIGHT_PWM = 23
    RIGHT_FORWARD = 25
    RIGHT_REVERSE = 24

    def __init__(
        self,
        gpio,
        frequency_hz=100.0,
        left_inverted=False,
        right_inverted=False,
    ):
        frequency_hz = float(frequency_hz)
        if not math.isfinite(frequency_hz) or not 1.0 <= frequency_hz <= 5000.0:
            raise ValueError("GPIO PWM frequency must be between 1 and 5000 Hz")
        self.gpio = gpio
        self.left_inverted = bool(left_inverted)
        self.right_inverted = bool(right_inverted)
        self.faulted = False
        self.closed = False
        self.left_pwm = None
        self.right_pwm = None

        for pin in (
            self.LEFT_PWM,
            self.LEFT_FORWARD,
            self.LEFT_REVERSE,
            self.RIGHT_PWM,
            self.RIGHT_FORWARD,
            self.RIGHT_REVERSE,
        ):
            self.gpio.setup_output(pin)
        self.left_pwm = self.gpio.create_pwm(self.LEFT_PWM, frequency_hz)
        self.right_pwm = self.gpio.create_pwm(self.RIGHT_PWM, frequency_hz)
        self.stop()

    def _best_effort_stop(self):
        first_error = None
        for pwm in (self.left_pwm, self.right_pwm):
            if pwm is None:
                continue
            try:
                pwm.ChangeDutyCycle(0.0)
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        for pin in (
            self.LEFT_FORWARD,
            self.LEFT_REVERSE,
            self.RIGHT_FORWARD,
            self.RIGHT_REVERSE,
        ):
            try:
                self.gpio.write(pin, False)
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        return first_error

    def stop(self):
        error = self._best_effort_stop()
        if error is not None:
            self.faulted = True
            raise MotorHardwareError("failed to clear direct GPIO motor outputs") from error

    def drive(self, left_duty, right_duty):
        if self.closed or self.faulted:
            raise MotorHardwareError("motor adapter is closed or fault-latched")
        try:
            left = -float(left_duty) if self.left_inverted else float(left_duty)
            right = -float(right_duty) if self.right_inverted else float(right_duty)
            self._drive_one(
                self.left_pwm,
                self.LEFT_FORWARD,
                self.LEFT_REVERSE,
                left,
            )
            self._drive_one(
                self.right_pwm,
                self.RIGHT_FORWARD,
                self.RIGHT_REVERSE,
                right,
            )
        except Exception as exc:
            self.faulted = True
            self._best_effort_stop()
            raise MotorHardwareError(
                "direct GPIO motor output failed; adapter latched off"
            ) from exc

    def _drive_one(self, pwm, forward_pin, reverse_pin, signed_duty):
        if not math.isfinite(signed_duty):
            raise ValueError("motor duty must be finite")
        signed_duty = max(-100.0, min(100.0, signed_duty))
        # Remove PWM before changing direction, matching the proven driver.
        pwm.ChangeDutyCycle(0.0)
        if abs(signed_duty) <= 1e-9:
            self.gpio.write(forward_pin, False)
            self.gpio.write(reverse_pin, False)
            return
        self.gpio.write(forward_pin, signed_duty > 0.0)
        self.gpio.write(reverse_pin, signed_duty < 0.0)
        pwm.ChangeDutyCycle(abs(signed_duty))

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._best_effort_stop()
        for pwm in (self.left_pwm, self.right_pwm):
            if pwm is not None:
                try:
                    pwm.stop()
                except Exception:
                    pass
        self.gpio.close()
