# Thruster Node (thruster_pkg)

## Launch multiple thruster nodes (recommended)

I added a launch file that spawns multiple driver nodes with per-node parameters.

- Launch default 4 thrusters (names: `thruster_0`..`thruster_3`):

```bash
ros2 launch thruster_pkg multi_thruster.launch.py
```

- Launch N thrusters with custom I2C settings and base name:

```bash
ros2 launch thruster_pkg multi_thruster.launch.py count:=6 base_name:=thruster i2c_bus:=7 i2c_address:=64
```

What this launch does
- Creates `count` nodes named `<base_name>_0` .. `<base_name>_{N-1}`.
- Sets each node's `topic` to `/<base_name>_<i>` and `channel` to `i` (so `thruster_2` -> channel 2).
- Sets `i2c_bus` and `i2c_address` for each node from the launch args.

Note: the launch expects the package's node executable to be available as `thruster_node`. If you run the script directly (not installed), either install the package or ask me to update the launch to run the script file path.

---

## Example end-to-end flow

1. Start joystick driver (or another source of `sensor_msgs/Joy`):

```bash
ros2 run joy joy_node
```

2. Run the thruster controller that maps joystick -> `/thruster_N` topics:

```bash
python3 src/controller/controller/thruster_controller_node.py
```

3. Launch thruster drivers (multi):

```bash
ros2 launch thruster_pkg multi_thruster.launch.py count:=4
```

You should now see `/thruster_0../thruster_3` being published by the controller and consumed by the respective driver nodes.

---

## Testing without hardware

- To inspect controller output without hardware:

```bash
ros2 topic echo /thruster_0
```

- To simulate a command to a thruster topic:

```bash
ros2 topic pub /thruster_0 std_msgs/msg/Float32 "{data: 0.5}" --once
```

- If you want a software-only test mode, I can add a `--dry-run` flag to the driver so it logs values instead of opening I2C.

---
