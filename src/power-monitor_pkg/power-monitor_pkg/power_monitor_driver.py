import smbus2
import time

# big endian

class INA226:
    """Driver for INA226 Power Monitor over I2C"""
    
    # Register addresses from datasheet
    REG_CONFIG = 0x00
    REG_SHUNT_VOLTAGE = 0x01
    REG_BUS_VOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05
    REG_MASK_ENABLE = 0x06
    REG_ALERT_LIMIT = 0x07
    REG_MANUFACTURER_ID = 0xFE
    REG_DIE_ID = 0xFF
    
    def __init__(self, i2c_address=0x4F, bus_number=7):
        """
        Args:
            i2c_address: I2C address of INA226 (default 0x40)
            bus_number: I2C bus number (usually 1 for Jetson)
        """
        self.address = i2c_address
        self.bus = smbus2.SMBus(bus_number)
        self.shunt_resistor = 0.004  # 0.1 ohm shunt resistor (CHECK YOUR HARDWARE)
        
    def _read_register(self, register):
        """Read 16-bit register from INA226"""
        data = self.bus.read_i2c_block_data(self.address, register, 2)
        # INA226 sends MSB first
        return (data[0] << 8) | data[1]
    
    def _write_register(self, register, value):
        """Write 16-bit value to register"""
        msb = (value >> 8) & 0xFF
        lsb = value & 0xFF
        self.bus.write_i2c_block_data(self.address, register, [msb, lsb])
    
    def configure(self):
        """Configure INA226 for continuous measurement"""
        # Default config: continuous shunt and bus voltage measurement
        # Averaging = 1, conversion time = 1.1ms for both
        config = 0x4127  # Default from datasheet
        self._write_register(self.REG_CONFIG, config)
        
    def calibrate(self, max_expected_current=17.0):
        """
        Calibrate the INA226 for current/power measurements
        
        Args:
            max_expected_current: Maximum expected current in Amps
        """
        # From datasheet: Current_LSB = max_expected_current / 32768
        current_lsb = max_expected_current / 32768.0
        
        # Cal = 0.00512 / (Current_LSB * Rshunt)
        cal_value = int(0.00512 / (current_lsb * self.shunt_resistor))
        self._write_register(self.REG_CALIBRATION, cal_value)
        
        self.current_lsb = current_lsb
        self.power_lsb = current_lsb * 25  # Power_LSB is 25x Current_LSB
    
    def read_shunt_voltage(self):
        """Read shunt voltage in millivolts"""
        raw = self._read_register(self.REG_SHUNT_VOLTAGE)
        # Handle two's complement for negative values
        if raw & 0x8000:
            raw = -((~raw & 0xFFFF) + 1)
        # LSB = 2.5 µV from datasheet
        return raw * 2.5e-6  # Return in Volts
    
    def read_bus_voltage(self):
        """Read bus voltage in Volts"""
        raw = self._read_register(self.REG_BUS_VOLTAGE)
        # LSB = 1.25 mV from datasheet
        return raw * 1.25e-3  # Return in Volts
    
    def read_current(self):
        """Read current in Amps"""
        raw = self._read_register(self.REG_CURRENT)
        # Handle two's complement
        if raw & 0x8000:
            raw = -((~raw & 0xFFFF) + 1)
        return raw * self.current_lsb
    
    def read_power(self):
        """Read power in Watts"""
        raw = self._read_register(self.REG_POWER)
        return raw * self.power_lsb
    
    def read_all(self):
        """Read all measurements at once"""
        return {
            'shunt_voltage': self.read_shunt_voltage(),
            'bus_voltage': self.read_bus_voltage(),
            'current': self.read_current(),
            'power': self.read_power()
        }