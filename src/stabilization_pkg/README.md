# Attitude stabilization

`stabilization_node` turns IMU orientation error and angular velocity into
bounded pitch/roll corrections. Houston mixes these corrections into joystick
commands when stabilization is enabled. The current implementation forces the
yaw correction to zero; the depth configuration section is reserved for future
work.

## Run

```sh
ros2 run stabilization_pkg stabilization_node
```

To load another configuration, append
`--ros-args -p stabilization_config:=/path/to/stabilization_config.yaml`.

## Topics

| Direction | Topic | Message | Purpose |
| --- | --- | --- | --- |
| Subscribe | `/imu` | `sensor_msgs/Imu` | Current orientation and gyro values |
| Subscribe | `/stabilizer/capture` | `std_msgs/Empty` | Capture current orientation as setpoint |
| Publish | `/stabilizer/commands` | `geometry_msgs/Twist` | Angular stabilization correction |

Publication follows IMU callbacks. `publish_rate_hz` is declared and read, but
currently does not create a timer. Linear components and yaw correction remain
zero. Invalid or near-zero orientation samples yield neutral corrections.

## Configuration and dependencies

[stabilization_config.yaml](config/stabilization_config.yaml) contains
`imu_stabilization.p`, `d`, `max_stabilization_influence`, and `use_setpoints`.
Defaults in that file are 0.5, 0, 0.5, and false. Capture requests only update the
setpoint when `use_setpoints` is true and a valid IMU orientation is available.
Setpoints are normalized; non-finite and near-zero values are rejected.

Runtime imports require ROS 2 `rclpy`, `geometry_msgs`, `std_msgs`, `sensor_msgs`,
`ament_index_python`, NumPy, and `nuwave_utils_pkg` with PyYAML.
See the [node source](stabilization_pkg/stabilization_node.py),
[IMU publisher source](../imu_pkg/imu_pkg/imu_node.py),
[software setup](../../software_setup.md), and
[ROS message definitions](https://github.com/ros2/common_interfaces).
