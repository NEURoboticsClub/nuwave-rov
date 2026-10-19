# Thruster Package (thruster_pkg)

## Purpose

The `pwm` node (`pwm_node` executable) drives every thruster and arm motor on the ROV from a single node.

Each motor in `config/individual_motor_config.yaml` has its own command topic. The node subscribes to each topic (`std_msgs/msg/Float32`), converts the command to a PWM pulse width using that motor's configured range, and writes it to the PWM hardware on that motor's `channel`. The pulse width actually sent is published back on `<topic>/response_pwm`.

The node also includes a watchdog that returns every motor to neutral if no command is received within the configured timeout.

## How to run

### With the launch file (recommended)
Loads the node parameters from `config/general_motor_config.yaml`:

```bash
ros2 launch thruster_pkg pwm.launch.py
```

### Directly
```bash
ros2 run thruster_pkg pwm_node
```

On the Jetson, set the board model first:

```bash
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
```

### Without hardware
Run in simulation mode to skip all hardware writes:

```bash
ros2 run thruster_pkg pwm_node --ros-args -p simulate:=true
```

## Configuration

| File | Contents |
| ---- | -------- |
| `config/general_motor_config.yaml` | Node parameters (`i2c_bus`, `i2c_address`, `pwm_freq_hz`, `watchdog_timeout_s`, …), loaded by `pwm.launch.py` |
| `config/individual_motor_config.yaml` | One entry per motor: `channel`, `topic`, and PWM ranges (`neutral_us`, `precision_*_us`, `speed_*_us`) |

Hardware is selected with the `pwm_hardware` parameter: `hat` (PCA9685 over I2C, the default) or `arduino` (Firmata over serial).

## Testing

- To watch the pulse width sent to a thruster:

```bash
ros2 topic echo /thruster/thruster_fll/response_pwm
```

- To send a command to a thruster:

```bash
ros2 topic pub /thruster/thruster_fll std_msgs/msg/Float32 "{data: 0.5}" --once
```
