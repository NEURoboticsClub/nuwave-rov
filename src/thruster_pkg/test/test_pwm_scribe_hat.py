"""Exercise the hat adapter and PCA9685 register writes without hardware."""

import importlib
from pathlib import Path
import sys
from types import ModuleType
from unittest.mock import Mock, patch

import pytest


class FakeSMBus:
    """Record I2C registers in memory; never open a device."""

    def __init__(self, bus):
        self.bus_number = bus
        self.registers = {}

    def write_byte_data(self, address, register, value):
        self.registers[address, register] = value

    def read_byte_data(self, address, register):
        return self.registers.get((address, register), 0)


@pytest.fixture
def hat():
    """Load the real adapter/driver with an isolated, fake smbus2 import."""
    package = ModuleType('_pwm_hat_test')
    package.__path__ = [str(Path(__file__).parents[1] / 'thruster_pkg')]
    smbus = ModuleType('smbus2')
    smbus.SMBus = FakeSMBus
    with patch.dict(sys.modules, {'_pwm_hat_test': package, 'smbus2': smbus}):
        module = importlib.import_module('_pwm_hat_test.pwm_scribe_hat')
        adapter = module.PWMScribeHat(bus=7, addr=0x40)
    adapter.setup(50)
    return adapter


@pytest.mark.parametrize('channel,pulse_us,expected_count', [
    (0, 1000, 204),
    (2, 1280, 262),
    (8, 1300, 266),
    (9, 1400, 286),
    (2, 1430, 292),
    (8, 1500, 307),
    (2, 1530, 313),
    (2, 1530.75, 313),
    (8, 1600, 327),
    (8, 1604, 328),
    (2, 1630, 333),
    (9, 1700, 348),
    (3, 1780, 364),
    (15, 2000, 409),
])
def test_microseconds_reach_driver_and_registers_unchanged(
        hat, monkeypatch, channel, pulse_us, expected_count):
    """Preserve calibrated pulse widths, allowing only 12-bit truncation."""
    pulse_writer = Mock(wraps=hat.board.setServoPulse)
    monkeypatch.setattr(hat.board, 'setServoPulse', pulse_writer)

    hat.set_pwm(channel, pulse_us)

    pulse_writer.assert_called_once_with(channel, pulse_us)
    registers = hat.board.bus.registers
    base = 0x06 + 4 * channel
    assert registers[0x40, base] == 0
    assert registers[0x40, base + 1] == 0
    count = registers[0x40, base + 2] | (registers[0x40, base + 3] << 8)
    assert count == expected_count
    assert 0 <= pulse_us - count * (20000 / 4096) < 20000 / 4096


def test_setup_uses_50_hz_prescaler(hat):
    """Match the direct pulse writer's documented 20 ms period."""
    assert hat.board.bus.registers[0x40, 0xFE] == 121
