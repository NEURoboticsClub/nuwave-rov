# Controller
This package functions as the controller for the ROV - converting joystick inputs to thrusters

Ideally, this node also performs some control algorithms, but not currently

## Requirements
This works together with the `controller_msgs` package. (Not currently, but hopefully)

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