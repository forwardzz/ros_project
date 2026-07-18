import pytest

from YbImuLib import YbImuI2c, YbImuSerial
from YbImuLib import YbImuI2cLib


def test_public_drivers_import_without_opening_hardware():
    assert YbImuSerial.__name__ == "YbImuSerial"
    assert YbImuI2c.__name__ == "YbImuI2c"


def test_i2c_driver_reports_missing_optional_dependency(monkeypatch):
    monkeypatch.setattr(YbImuI2cLib, "SMBus", None)

    with pytest.raises(RuntimeError, match="smbus2"):
        YbImuI2c()
