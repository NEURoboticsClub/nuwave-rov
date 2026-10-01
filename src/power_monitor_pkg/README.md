# Power Monitor

## Purpose

The `power_monitor_pkg` package reads bus voltage, current, power, and shunt voltage from INA226 power monitors through I2C. The node `power_monitor_pub` publishes each reading as its own ROS 2 topic.

Measurements are published at a fixed rate. There is also a launch file that starts one node for each monitor listed in a YAML config file. If the INA226 can't be set up when the node starts, an error is logged and it exits. If a read fails while the node is running, it logs the error once and won't log it again until a read works. 

## How to run
### Running multiple monitors with the launch file
```
ros2 launch power_monitor_pkg multi_power_monitor.launch.py
```

### Using a custom config
```
ros2 launch power_monitor_pkg multi_power_monitor.launch.py \
  config:=/path/to/power_monitor_run_config.yaml
```

### Running a single monitor
```
ros2 run power_monitor_pkg power_monitor_pub \
  --ros-args \
  -p i2c_address:=0x4F \
  -p i2c_bus:=1 \
  -p topic:=power_monitor
```
One or more of the above parameters are optional

## Subscribed Topics

This node doesn't subscribe to any topics.

## Published Topics

Every topic starts with the `topic` parameter (default `power_monitor`).

| Topic                   | Type                   | Description                 |
| ----------------------- | ---------------------- | --------------------------- |
| `<topic>/bus_voltage`   | `std_msgs/msg/Float32` | Bus voltage in volts.       |
| `<topic>/current`       | `std_msgs/msg/Float32` | Current in amps.            |
| `<topic>/power`         | `std_msgs/msg/Float32` | Power in watts.             |
| `<topic>/shunt_voltage` | `std_msgs/msg/Float32` | Shunt voltage in volts.     |

## Parameters

| Parameter              |         Default | Description                                                                                |
| ---------------------- | --------------: | ------------------------------------------------------------------------------------------ |
| `i2c_address`          |          `0x4F` | I2C address of the INA226.                                                                 |
| `i2c_bus`              |             `1` | I2C bus number the INA226 is connected to.                                                 |
| `shunt_resistor`       |         `0.004` | The node reads this without passing it to the driver since the driver always uses `0.004`. |
| `max_expected_current` |          `17.0` | The node reads this but the calibration always uses `17.0`.                                |
| `publish_rate_hz`      |          `10.0` | The node reads this but the timer is always set to 10 Hz.                                  |
| `topic`                | `power_monitor` | Start of the name for all of the published topics.                                         |
| `topic_prefix`         | `power_monitor` | Declared in the node but never used. Use `topic` instead.                                  |

## Configuration

The INA226 is configured and calibrated once when the node starts. The I2C address and bus are set with the `i2c_address` and `i2c_bus` parameters.

The launch file gets its list of monitors from `power_monitor_run_config.yaml` which defaults to be in the package's `config` folder. Each entry under `power_monitor` has:
* `id`
* `i2c_address`
* `i2c_bus`

## Methods

| Method             |  Description                                                                                                    |
| ------------------ | --------------------------------------------------------------------------------------------------------------- |
| `__init__()`       | Sets up the node, parameters, publishers, and INA226 before starting the timer. Raises `PowerMonitorInitError` if the INA226 setup fails.                                                                                                                           |
| `timer_callback()` | Reads all the measurements from the INA226 and publishes them. If a read fails, it logs the error.                                                                                                                                 |
| `main()`           | Starts ROS 2, creates the node, and runs it. If the node cannot be set up, it shuts down and exits.                                                                                                                                 |

## Dependencies

### ROS 2 Packages

* `rclpy`
* `std_msgs`

### Python Packages

* `smbus2` — used by the INA226 driver to talk over I2C.

### Project Packages

* `power_monitor_pkg` — the INA226 driver (`power_monitor_pkg.power_monitor_driver`) is in this same package.


## External Documentation

* [std_msgs/msg/Float32](https://docs.ros.org/en/rolling/p/std_msgs/msg/Float32.html)