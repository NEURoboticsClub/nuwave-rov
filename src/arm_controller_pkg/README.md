# Arm controller

`arm_controller_node` maps the six values in an arm command array to the motor
IDs and topics in [arm_config.yaml](config/arm_config.yaml). The timer publishes
normalized motor commands at 50 Hz by default. PWM conversion happens downstream
in `thruster_pkg`.

## Run

```sh
ros2 run arm_controller_pkg arm_controller_node
```

Use `--ros-args -p arm_config:=/path/to/arm_config.yaml` for another motor map.

## Topics

| Direction | Topic | Message and meaning |
| --- | --- | --- |
| Subscribe | `arm_commands` | `std_msgs/Float32MultiArray`, ordered below |
| Publish | `/arm/motor_base_yaw` | `std_msgs/Float32`, normalized command |
| Publish | `/arm/motor_base_pitch` | Same |
| Publish | `/arm/motor_elbow_pitch` | Same |
| Publish | `/arm/motor_wrist_yaw` | Same |
| Publish | `/arm/motor_wrist_pitch` | Same |
| Publish | `/arm/motor_claw` | Same |

Array order: base yaw, base pitch, elbow pitch, wrist yaw, wrist pitch, claw.
Configuration IDs are one-based indexes into that array; publication order follows
the configuration. Missing or non-finite command values become zero. Before the
first command, and after `watchdog_timeout_s` (default 0.5 s), outputs are zero.
Downstream PWM nodes map zero to each motor's configured neutral pulse.

## Parameters and dependencies

`publish_rate_hz` defaults to 50; `arm_topic` defaults to `arm_commands`;
`arm_config` defaults to this package's installed config file.
Runtime imports require ROS 2 `rclpy`, `geometry_msgs`, `std_msgs`,
`ament_index_python`, NumPy, and `nuwave_utils_pkg` with PyYAML.
See the [source](arm_controller_pkg/arm_controller_node.py),
[software setup guide](../../software_setup.md), and
[ROS message definitions](https://github.com/ros2/common_interfaces).
