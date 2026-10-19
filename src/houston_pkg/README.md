# Houston

## Purpose

The `houston` node is the control hub for ROV operations. It takes input from two gamepad controllers, one for driving the vehicle and one for operating the arm, and turns them into commands for the rest of the system.

Controller axes and buttons are mapped to named controls through a YAML config file (`joystick_config.yaml`), which also sets per-axis inversion, sensitivity, and scaling. Drive input is converted into a velocity command (`geometry_msgs/msg/Twist`) published at a fixed rate, and arm input is converted into a 6-element array of joint commands published each time new arm input arrives.

The node also manages the ROV's control modes: exponential controls, precision mode, and stabilization. Each mode can be toggled from either the controller or the GUI, and its current state is published so other nodes stay in sync. When stabilization is enabled, the node adds corrections from the stabilizer node to the velocity command, and it turns stabilization off automatically if those corrections stop arriving.

The node includes a watchdog that publishes a zero velocity command if drive input is not received within the configured timeout.

## Requirements
Uses ROS2 built-in Joy package, requires ros-humble-joy dependency, along with all imports listed at top of file and in this README's "Dependencies" section.  

## How to run
### Running with default configs
```
ros2 run houston_pkg houston_node
```

### Creating new config
```
ros2 run houston_pkg joystick_identify_node --ros-args -p joy_topic:=/joy_thruster -p output_file:=/path/to/new_joystick_config.yaml
```

### Using a custom config
```
ros2 run houston_pkg houston_node \
  --ros-args \
  -p joy_config:=/path/to/joystick_config.yaml \
  -p joy_thruster:=/joy_thruster \
  -p joy_arm:=/joy_arm
```
One or more of the above params are optional


## Subscribed Topics

| Topic                           | Type                  | Description                                                                            |
| ------------------------------- | --------------------- | -------------------------------------------------------------------------------------- |
| `joy_thruster`                  | `sensor_msgs/Joy`     | Thruster joystick input, processed in the publish loop into `velocity_commands`        |
| `joy_arm`                       | `sensor_msgs/Joy`     | Arm joystick input, parsed on arrival and published to `arm_commands`                  |
| `stabilizer/commands`           | `geometry_msgs/Twist` | Stabilizer correction twist, added to joystick commands when stabilization is enabled  |
| `gui_buttons/expo_enabled`      | `std_msgs/Bool`       | Sets exponential controls on/off from the GUI                                          |
| `gui_buttons/precision_mode`    | `std_msgs/Bool`       | Sets precision mode on/off from the GUI                                                |
| `gui_buttons/stabilize_enabled` | `std_msgs/Bool`       | Sets stabilization on/off from the GUI                                                 |

## Published Topics

| Topic                        | Type                         | Description                                                                                               |
| ---------------------------- | ---------------------------- | --------------------------------------------------------------------------------------------------------- |
| `velocity_commands`          | `geometry_msgs/Twist`        | Twist from the thruster joystick, plus stabilizer output when enabled. Zeroed if joystick input is stale  |
| `arm_commands`               | `std_msgs/Float32MultiArray` | 6-element array of arm inputs: base yaw, base pitch, elbow pitch, wrist yaw, wrist up/down, claw          |
| `controls/expo_enabled`      | `std_msgs/Bool`              | Current exponential controls state                                                                        |
| `controls/precision_mode`    | `std_msgs/Bool`              | Current precision mode state                                                                              |
| `controls/stabilize_enabled` | `std_msgs/Bool`              | Current stabilization state                                                                               |
| `stabilizer/capture`         | `std_msgs/Empty`             | Trigger sent to the stabilizer whenever stabilization is turned on                                        |


## Parameters

| Parameter               | Default                                                   | Description                                                                               |
| ------------------------| ----------------------------------------------------------| ----------------------------------------------------------------------------------------- |
| `joy_config`            |`os.path.join(pkg_share, 'config', 'joystick_config.yaml'` | File path for config YAML                                                                 |
| `stabilizer_timeout`    |                                                     `0.5` | Time in seconds before stabilizer data is considered stale and stabilization is disabled  |
| `joy_thruster_timeout`  |                                                     `0.5` | Time in seconds before thruster joystick data is considered stale and zero the twist      |
| `publish_rate_hz`       |                                                    `50.0` | Publish rate in hertz of houston twist commands                                           |
| `expo_enabled_default`  |                                                   `False` | Default state of `expo_enabled` topics, controlling exponential control                   |
| `precision_mode_default`|                                                   `False` | Default state of `precision_enabled` topics, controlling precision control                |


## Configuration

The `joystick_config.yaml` defines the inputs for the ROV and the Arm controllers (Xbox, not PID):

If Axis Input (Joystick):
* Axis Name
* Input
* Invert
* Sensitivity
* Scale

If Button Input:
* Button Name
* Input
* Invert

## Functions

| Function                               | Description                                                                                                        |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `__init__()`                           | Initializes the node, parameters, configurations, publishers, subscribers, watchdog, and internal states           |
| `publish_expo_state`                   | Publishes current `expo_enabled` value to `/controls/expo_enabled` topic                                           |
| `set_expo_enabled`                     | Updates `expo_enabled` and publishes the new state if it changed                                                   |
| `gui_expo_toggle_callback`             | Sets `expo_enabled` from the `/gui_buttons/expo_enabled` topic                                                     |
| `publish_precision_mode_state`         | Publishes current `precision_mode_enabled` value to `/controls/precision_mode` topic                               |
| `set_precision_mode_enabled`           | Updates `precision_mode_enabled` and publishes the new state if it changed                                         |
| `gui_precision_mode_toggle_callback`   | Sets `precision_mode_enabled` from the `/gui_buttons/precision_mode` topic                                         |
| `publish_stabilize_state`              | Publishes current `stabilize_enabled` value to `/controls/stabilize_enabled` topic                                 |
| `set_stabilize_enabled`                | Updates `stabilize_enabled` and publishes the new state if it changed. Triggers `/stabilizer/capture` on enable    |
| `gui_stabilize_toggle_callback`        | Sets `stabilize_enabled` from the `/gui_buttons/stabilize_enabled` topic                                           |
| `scale_controller_input`               | Applies a normalized exponential curve (with a small center deadzone) to a joystick input in [-1, 1]               |
| `stabilizer_callback`                  | Stores the latest `Twist` from `/stabilizer/commands` and its timestamp                                            |
| `joy_arm_callback`                     | Parses arm joystick input and passes it to `publish_arm_commands`                                                  |
| `joy_thruster_callback`                | Stores the latest thruster joystick message and its timestamp                                                      |
| `_publish_loop`                        | Timer callback that parses thruster input and publishes the twist, or a neutral twist if input is stale            |
| `parse_joystick`                       | Maps raw `Joy` axes/buttons to named values from the config, applying invert, deadzone, sensitivity, and scale     |
| `publish_twist`                        | Handles mode toggle buttons, builds the `Twist` (adding stabilizer output if enabled), and publishes it            |
| `publish_arm_commands`                 | Builds and publishes a 6-element `Float32MultiArray` to `arm_commands`                                             |
| `main`                                 | Initializes ROS 2, spins the `Houston` node, and shuts down on exit                                                |


## Dependencies

### ROS 2 Packages

* `rclpy`
* `sensor_msgs`
* `geometry_msgs`
* `std_msgs`
* `ament_index_python`

### Python Packages

* `numpy`

### Project Packages

* `nuwave_utils_pkg` — used to load the controller configuration YAML.
