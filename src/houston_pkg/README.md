# Houston
This package functions as the control hub for the full ROV operations. It receives two controller (like Xbox controller, not PID) inputs; one for drive controls, one for arm controls, then publishes and subscribes to various topics to coordinate other nodes' functionalities. 

## Requirements
Uses ROS2 built-in Joy package, requires ros-humble-joy dependency, along with all imports listed at top of file.  

## How to run
### Running with default configs
```
ros2 run houston_pkg houston
```
## Topics
### Publishing
- geometry_msgs/Twist data on topic /velocity_commands
- Float32MultiArray data on topic /arm_commands
- std_msgs/Bool data on topic /controls/expo_enabled
- std_msgs/Bool data on topic /controls/precision_mode
- std_msgs/Bool data on topic /controls/stabilize_enabled
- std_msgs/Bool data on topic /stabilizer/capture

### Subscribing 
- sensor_msgs/joy data on topic joy_thruster
- sensor_msgs/joy data on topic joy_arm
- geometry_msgs/Twist data on topic /stabilizer/commands
- std_msgs/Bool data on /gui_buttons/expo_enabled
- std_msgs/Bool data on /gui_buttons/precision_mode
- std_msgs/Bool data on /gui_buttons/stabilize_enabled

## Configs
- joy_config
    - Parameter containing YAML path. The YAML file contains the button mapping to control commands. 
- stabilizer_timeout
    - Parameter, in seconds; delay until stabilizer data is considered stale and stabilization is disabled
- joy_thruster_timeout
    - Parameter, in seconds; delay until thruster joystick data is considered stale and twist is zeroed
- publish_rate_hz
    - Parameter, in Hertz; publish rate of houston twist commands
- expo_enabled_default
    - Parameter, bool; Default state of /expo_enabled topics
- precision_mode_default
    - Parameter, bool; Default state of /precision_enabled topics

### Creating new config
```
ros2 run houston_pkg joystick_identify --ros-args -p joy_topic:=/joy_thruster -p output_file:=/path/to/new_joystick_config.yaml
```

### Using a custom config
```
ros2 run houston_pkg houston \
  --ros-args \
  -p joy_config:=/path/to/joystick_config.yaml \
  -p joy_thruster:=/joy_thruster \
  -p joy_arm:=/joy_arm
```
One or more of the above params are optional
