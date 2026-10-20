# IMU publisher

`imu_node` reads a BNO08x IMU and publishes orientation, angular velocity, and
linear acceleration in the ROV body frame. The mount quaternion corrects for the
sensor's installed orientation before values are sent to the stabilizer and GUI.

## Run

```sh
ros2 run imu_pkg imu_node --ros-args -p addr:=7 -p sampling_rate:=100
```

Despite its name, `addr` selects the ExtendedI2C bus number; it is not the sensor's
I2C device address. Startup retries driver creation after failures. A failed or
incomplete sensor read is logged and skipped for that cycle.

## Topics

| Direction | Topic | Message | Contents |
| --- | --- | --- | --- |
| Publish | `imu` | `sensor_msgs/Imu` | Quaternion, angular velocity (rad/s), acceleration (m/s^2) |

There are no subscribed topics. The message timestamp comes from the ROS clock
and its frame ID is `imu_link`. The publisher uses the driver's game quaternion;
its yaw reference depends on that sensor mode.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `addr` | 7 | I2C bus number |
| `sampling_rate` | 100 | Timer rate in Hz |
| `q_mount` | `[0, 0, -0.7071068, 0.7071068]` | Mount quaternion in ROS x,y,z,w order |

Vectors are rotated by the mount quaternion. Published orientation is the chip
quaternion multiplied by the inverse mount quaternion.

## Dependencies and references

Runtime imports require ROS 2 `rclpy`, `sensor_msgs`, `geometry_msgs`, NumPy,
Adafruit Blinka (`board`/`busio`), `adafruit-extended-bus`, and
`adafruit-circuitpython-bno08x`, plus the configured I2C hardware.
See the [node](imu_pkg/imu_node.py), [driver](imu_pkg/imu_driver.py),
[Adafruit BNO08x documentation](https://docs.circuitpython.org/projects/bno08x/en/latest/),
and [software setup](../../software_setup.md).
