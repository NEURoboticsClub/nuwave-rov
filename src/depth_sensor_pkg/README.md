# Depth sensor

`depth_sensor_node` reads an MS5837-02BA pressure/temperature sensor over I2C.
It publishes the latest depth plus a recent-reading history for the dashboard.
Depth uses the driver's pressure reference and fluid density; verify those
assumptions for the actual sensor and water before interpreting measurements.

## Run

```sh
ros2 run depth_sensor_pkg depth_sensor_node --ros-args -p i2c_bus:=1
```

Startup retries sensor initialization once per second until it succeeds.
Failed reads are logged and skipped for that timer cycle.

## Topics

| Direction | Topic | Message | Units |
| --- | --- | --- | --- |
| Publish | `depth/depth` | `std_msgs/Float32` | metres |
| Publish | `depth/depth_array` | `std_msgs/Float32MultiArray` | metres |
| Publish | `depth/pressure` | `sensor_msgs/FluidPressure` | pascals |
| Publish | `depth/temperature` | `sensor_msgs/Temperature` | degrees C |

There are no subscribed topics. Pressure and temperature messages carry the
current ROS timestamp and frame ID `depth_sensor`. Pressure is converted from
mbar to Pa before publication. The history contains readings, rather than a
computed average.

## Parameters and dependencies

`publish_rate` defaults to 10 Hz; `depth_list_length` controls the history limit
(default 10); `i2c_bus` defaults to 1.
Runtime requirements are ROS 2 `rclpy`, `sensor_msgs`, `std_msgs`, `smbus2`, and
an accessible I2C bus with the matching sensor.
See the [node source](depth_sensor_pkg/depth_sensor_node.py),
[local driver](depth_sensor_pkg/ms5837_driver.py),
[upstream MS5837 documentation](https://github.com/bluerobotics/ms5837-python),
and [software setup](../../software_setup.md).
