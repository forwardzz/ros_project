import pytest

from inspection_robot_hardware.motor_board import (
    BcmGpio,
    DirectGpioMotorBoard,
    MotorHardwareError,
    Pca9685,
    V40MotorBoard,
)


class FakePca:
    def __init__(self):
        self.events = []
        self.fail_channel = None
        self.fail_once = False
        self.closed = False

    def set_duty_cycle(self, channel, duty):
        if self.fail_once and channel == self.fail_channel:
            self.fail_once = False
            raise OSError("simulated I2C write failure")
        self.events.append((channel, duty))

    def close(self):
        self.closed = True


class FakeGpio:
    def __init__(self):
        self.configured = []
        self.events = []
        self.pwms = {}
        self.closed = False

    def setup_output(self, pin):
        self.configured.append(pin)

    def write(self, pin, value):
        self.events.append((pin, value))

    def create_pwm(self, pin, frequency_hz):
        pwm = FakePwm(pin, frequency_hz)
        self.pwms[pin] = pwm
        return pwm

    def close(self):
        self.closed = True


class FakePwm:
    def __init__(self, pin, frequency_hz):
        self.pin = pin
        self.frequency_hz = frequency_hz
        self.events = [("start", 0.0)]
        self.fail_once = False
        self.stopped = False

    def ChangeDutyCycle(self, duty):
        if self.fail_once:
            self.fail_once = False
            raise OSError("simulated GPIO PWM failure")
        self.events.append(("duty", duty))

    def stop(self):
        self.stopped = True


class FakeSmbus:
    def __init__(self, bus_number):
        self.bus_number = bus_number
        self.byte_writes = []
        self.block_writes = []
        self.closed = False

    def read_byte_data(self, address, register):
        assert (address, register) == (0x40, 0x00)
        return 0x01

    def write_byte_data(self, address, register, value):
        self.byte_writes.append((address, register, value))

    def write_i2c_block_data(self, address, register, values):
        self.block_writes.append((address, register, values))

    def close(self):
        self.closed = True


class FakeGpioModule:
    BCM = 11
    OUT = 1
    LOW = 0
    HIGH = 1

    def __init__(self):
        self.setup_calls = []
        self.output_calls = []
        self.cleanup_calls = []

    def setwarnings(self, _enabled):
        pass

    def setmode(self, mode):
        assert mode == self.BCM

    def setup(self, pin, mode, initial=None):
        self.setup_calls.append((pin, mode, initial))

    def output(self, pin, value):
        self.output_calls.append((pin, value))

    def cleanup(self, pins):
        self.cleanup_calls.append(pins)


def test_ab_preset_and_direction_outputs():
    pca = FakePca()
    board = V40MotorBoard("ab", pca)
    pca.events.clear()

    board.drive(30.0, -40.0)

    assert pca.events == [
        (0, 0.0),
        (2, 100.0),
        (1, 0.0),
        (0, 30.0),
        (5, 0.0),
        (3, 0.0),
        (4, 100.0),
        (5, 40.0),
    ]


def test_cd_uses_validated_direct_gpio_interface_and_inversion():
    gpio = FakeGpio()
    board = DirectGpioMotorBoard(gpio, left_inverted=True)
    gpio.events.clear()
    for pwm in gpio.pwms.values():
        pwm.events.clear()

    board.drive(20.0, 25.0)

    assert gpio.configured == [18, 22, 27, 23, 25, 24]
    assert gpio.pwms[18].frequency_hz == 100.0
    assert gpio.pwms[23].frequency_hz == 100.0
    assert gpio.pwms[18].events == [
        ("duty", 0.0),
        ("duty", 20.0),
    ]
    assert gpio.pwms[23].events == [
        ("duty", 0.0),
        ("duty", 25.0),
    ]
    assert gpio.events == [
        (22, False),
        (27, True),
        (25, True),
        (24, False),
    ]


def test_output_error_latches_off_and_clears_all_channels():
    pca = FakePca()
    board = V40MotorBoard("ab", pca)
    pca.events.clear()
    pca.fail_channel = 3
    pca.fail_once = True

    with pytest.raises(MotorHardwareError):
        board.drive(30.0, 30.0)

    assert board.faulted
    last_value = {}
    for channel, duty in pca.events:
        last_value[channel] = duty
    assert last_value == {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}
    with pytest.raises(MotorHardwareError):
        board.drive(10.0, 10.0)


def test_direct_gpio_error_latches_off_and_clears_outputs():
    gpio = FakeGpio()
    board = DirectGpioMotorBoard(gpio)
    gpio.pwms[23].fail_once = True

    with pytest.raises(MotorHardwareError):
        board.drive(20.0, 20.0)

    assert board.faulted
    assert gpio.pwms[18].events[-1] == ("duty", 0.0)
    assert gpio.pwms[23].events[-1] == ("duty", 0.0)
    assert gpio.events[-4:] == [
        (22, False),
        (27, False),
        (25, False),
        (24, False),
    ]


def test_close_stops_direct_gpio_outputs_and_releases_backend():
    gpio = FakeGpio()
    board = DirectGpioMotorBoard(gpio)
    board.drive(20.0, 20.0)
    gpio.events.clear()

    board.close()

    assert gpio.closed
    assert gpio.pwms[18].stopped and gpio.pwms[23].stopped
    assert gpio.pwms[18].events[-1] == ("duty", 0.0)
    assert gpio.pwms[23].events[-1] == ("duty", 0.0)
    assert (22, False) in gpio.events
    assert (27, False) in gpio.events
    assert (25, False) in gpio.events
    assert (24, False) in gpio.events


def test_pca9685_uses_i2c1_address_0x40_and_100_hz():
    buses = []

    def factory(bus_number):
        bus = FakeSmbus(bus_number)
        buses.append(bus)
        return bus

    pca = Pca9685(bus_factory=factory, channels_to_clear=(0, 5))
    bus = buses[0]

    assert bus.bus_number == 1
    assert bus.block_writes[0] == (0x40, 0x06, [0x00, 0x00, 0x00, 0x10])
    assert bus.block_writes[1] == (0x40, 0x1A, [0x00, 0x00, 0x00, 0x10])
    assert (0x40, 0xFE, 60) in bus.byte_writes
    pca.set_duty_cycle(5, 50.0)
    assert bus.block_writes[-1][0:2] == (0x40, 0x1A)
    pca.close()
    assert bus.closed


def test_bcm_gpio_drives_low_during_setup_and_close():
    module = FakeGpioModule()
    gpio = BcmGpio(module)
    gpio.setup_output(25)
    gpio.write(25, True)
    gpio.close()

    assert module.setup_calls == [(25, module.OUT, module.LOW)]
    assert module.output_calls[-1] == (25, module.LOW)
    assert module.cleanup_calls == [[25]]
