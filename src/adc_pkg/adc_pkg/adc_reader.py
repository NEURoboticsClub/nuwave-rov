#!/usr/bin/env python3
"""
ROS2 node for reading analog data from an AD7991YRJZ-1500RL7
on a Jetson Orin Nano.

Hardware setup:
  - AD7991 (12-bit ADC), -1 version -> I2C address 0x29
  - 3 channels: VIN0, VIN1, VIN2
  - External 3.3V reference on VIN3/VREF
  - Jetson Orin Nano, I2C bus 7 (40-pin header pins 3/5)

    *** IMPORTANT: Verify your I2C bus number! Run:
        sudo i2cdetect -l
        sudo i2cdetect -y -r 7
    The bus number can change between JetPack versions.
    Pins 3/5 are typically bus 7, pins 27/28 are typically bus 1. ***

Wiring:
  Jetson Pin 1  (3.3V)  -> AD7991 VDD (pin 8) + VIN3/VREF (pin 6)
  Jetson Pin 6  (GND)   -> AD7991 GND (pin 7)
  Jetson Pin 3  (SDA)   -> AD7991 SDA (pin 2)  [+ pull-up to 3.3V]
  Jetson Pin 5  (SCL)   -> AD7991 SCL (pin 1)  [+ pull-up to 3.3V]
  Analog signals         -> VIN0 (pin 3), VIN1 (pin 4), VIN2 (pin 5)
  1uF decoupling cap on VIN3/VREF recommended per datasheet.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import smbus2

# Hardware constants for AD7991YRJZ-1500RL7
I2C_BUS = 7              # Jetson Orin Nano 40-pin header pins 3/5
I2C_ADDR = 0x29          # -1 version address
VREF = 3.3               # External reference on VIN3/VREF
ADC_COUNTS = 4096        # 12-bit: 2^12
NUM_CHANNELS = 3         # VIN0, VIN1, VIN2
READ_HZ = 10.0           # Publishing rate

# Config register: enable CH0-CH2, external ref, filtering & delays on
# D7=0 D6=1 D5=1 D4=1 D3=1 D2=0 D1=0 D0=0
#  CH3  CH2  CH1  CH0  REF  FLTR BTD   SD
CONFIG_BYTE = 0x78


class AD7991Reader(Node):
    def __init__(self):
        super().__init__('ad7991_reader')

        # Open I2C -- force=True in case the Jetson kernel claims the address
        try:
            self.bus = smbus2.SMBus(I2C_BUS, force=True)
            self.get_logger().info(
                f'Opened /dev/i2c-{I2C_BUS}, device 0x{I2C_ADDR:02X}'
            )
        except Exception as e:
            self.get_logger().fatal(f'Cannot open I2C bus {I2C_BUS}: {e}')
            raise

        # Write config register (single-byte write per datasheet Figure 24)
        try:
            self.bus.write_byte(I2C_ADDR, CONFIG_BYTE)
            self.get_logger().info(f'Config written: 0x{CONFIG_BYTE:02X}')
        except Exception as e:
            self.get_logger().fatal(f'Config write failed: {e}')
            raise

        self.pub = self.create_publisher(Float32MultiArray, 'ad7991/voltages', 10)
        self.timer = self.create_timer(1.0 / READ_HZ, self._read_and_publish)
        self.get_logger().info('AD7991 node running.')

    def _read_conversion(self):
        """
        Read one 2-byte conversion result.
        Returns (channel_id, voltage) or None on failure.

        Byte format (Table 12/13):
          [0, 0, CHID1, CHID0, D11..D8] [D7..D0]
        """
        try:
            data = self.bus.read_i2c_block_data(I2C_ADDR, 0, 2)
            raw = (data[0] << 8) | data[1]
            ch = (raw >> 12) & 0x03
            code = raw & 0x0FFF
            return ch, (code / ADC_COUNTS) * VREF
        except Exception:
            return None

    def _read_and_publish(self):
        """Read 3 channels (sequenced automatically) and publish."""
        voltages = [0.0, 0.0, 0.0]

        for _ in range(NUM_CHANNELS):
            result = self._read_conversion()
            if result:
                ch, v = result
                if ch < NUM_CHANNELS:
                    voltages[ch] = v

        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='channel', size=NUM_CHANNELS, stride=NUM_CHANNELS)
        ]
        msg.data = voltages
        self.pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'bus'):
            self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AD7991Reader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down adc_reader node...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()