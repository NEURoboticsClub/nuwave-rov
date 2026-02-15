# Setup

## Verify the ADC is visible on I2C
sudo i2cdetect -l

sudo i2cdetect -y -r 7

### Expected result: 0x29
If not, check wiring or use a different bus number.

## Install smbus2 if not already
pip3 install smbus2

# Run ROS and node

cd ~/nuwave-rov

colcon build --packages-select adc_pkg

source install/setup.sh

ros2 run adc_pkg adc_reader

### (In a second terminal) Verify data is publishing

source ~/nuwave-rov/install/setup.bash

ros2 topic echo /ad7991/voltages