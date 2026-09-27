# Thruster Controller

## Purpose

The `thruster_controller` node converts a desired vehicle motion command (`geometry_msgs/msg/Twist`) into individual normalized commands for each configured thruster.

It uses the position and direction of each thruster to calculate a thruster allocation matrix, which is used to determine the force required from each thruster. Commands are published at a fixed rate.

The node also includes a watchdog that sets all thruster commands to zero if no velocity command is received within the configured timeout.

## Subscribed Topics

| Topic               | Type                      | Description                                                                    |
| ------------------- | ------------------------- | ------------------------------------------------------------------------------ |
| `velocity_commands` | `geometry_msgs/msg/Twist` | Desired linear and angular vehicle motion. Configurable with `thruster_topic`. |

## Published Topics

The node publishes one topic for each thruster defined in `thruster_config.yaml`.

| Type                   | Description                                                                  |
| ---------------------- | ---------------------------------------------------------------------------- |
| `std_msgs/msg/Float32` | Normalized command for an individual thruster, ranging from `-1.0` to `1.0`. |

The topic for each thruster is specified by its `topic` field in the thruster configuration.

## Parameters

| Parameter            |             Default | Description                                                    |
| -------------------- | ------------------: | -------------------------------------------------------------- |
| `max_x_n`            |         `4/sqrt(2)` | Maximum X force used for scaling.                              |
| `max_y_n`            |         `4/sqrt(2)` | Maximum Y force used for scaling.                              |
| `max_z_n`            |               `4.0` | Maximum Z force used for scaling.                              |
| `max_roll_nm`        |               `1.0` | Maximum roll torque used for scaling.                          |
| `max_pitch_nm`       |               `1.0` | Maximum pitch torque used for scaling.                         |
| `max_yaw_nm`         |               `1.0` | Maximum yaw torque used for scaling.                           |
| `max_force_n`        |               `1.0` | Force corresponding to a normalized thruster command of `1.0`. |
| `publish_rate_hz`    |              `50.0` | Thruster command publishing frequency.                         |
| `thruster_config`    | Package config path | Path to the thruster configuration YAML file.                  |
| `thruster_topic`     | `velocity_commands` | Input velocity command topic.                                  |
| `watchdog_timeout_s` |               `0.5` | Time without a command before outputs are set to zero.         |

## Configuration

The `thruster_config.yaml` file defines each thruster's:

* Position in meters
* Direction vector
* Output topic

The direction vector is normalized by the node before calculating the allocation matrix.

## Methods

| Method                                 | Description                                                                                                        |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `__init__()`                           | Initializes the node, parameters, thruster configuration, publishers, subscriber, allocation matrix, and watchdog. |
| `compute_thruster_allocation_matrix()` | Builds the allocation matrix from the position and direction of each thruster.                                     |
| `Status_Callback()`                    | Receives a `Twist` command, converts it to thruster forces, and stores the resulting commands.                     |
| `map_twist_to_toque()`                 | Converts the requested linear and angular motion into individual thruster forces using the allocation matrix.      |
| `map_torque_to_dutycycles()`           | Converts thruster forces into normalized commands between `-1` and `1`.                                            |
| `publish_thrusters()`                  | Publishes the current command for each thruster and applies the watchdog timeout.                                  |
| `main()`                               | Initializes ROS 2, starts the node, and runs the ROS 2 event loop.                                                 |

## Dependencies

### ROS 2 Packages

* `rclpy`
* `geometry_msgs`
* `std_msgs`
* `ament_index_python`

### Python Packages

* `numpy`

### Project Packages

* `nuwave_utils_pkg` — used to load the thruster configuration YAML.


## How to run
### Running with default configs
```
ros2 run controller thruster_controller_node
```

### Creating new config
```
ros2 run controller joystick_identify
```

### Using a custom config
```
ros2 run controller thruster_controller \
  --ros-args \
  -p joy_config:=/path/to/joystick_config.yaml \
  -p thruster_config:=/path/to/thruster_config.yaml \
  -p joy_topic:=/joy \
  -p thruster_topic:=/thruster
```
One or more of the above params are optional


## External Documentation

* [geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)
* [std_msgs/msg/Float32](https://docs.ros.org/en/rolling/p/std_msgs/msg/Float32.html)



