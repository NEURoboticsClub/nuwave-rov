# mate_rov_sim


## Gazebo Install
sudo apt-get update && sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic


Gazebo Sim (Harmonic) simulation of the NUWave ROV, driven by the **real NUWave
software**: the same gamepads, houston, thruster controller, arm controller,
pwm_node, stabilization and web GUI you run on the robot. Only the hardware
drivers are swapped for simulator stand-ins.

```
gamepads ─▶ joy_node ×2 ─────┐
keyboard ─▶ keyboard_teleop ─┴▶ joy_mux ─▶ houston ─▶ velocity_commands ─▶ controller/thruster_controller_node ─┐
                                                │      ─▶ arm_commands      ─▶ arm_controller/arm_controller_node ─┤
                                                └─◀ /stabilizer/commands ◀─ stabilization_node ◀─ /imu ◀─┐          │
                                                                                      │          ▼
          /thruster/*, /arm/motor_*  (duty cycle -1..1)  ─▶  thruster_pkg/pwm_node  (simulate:=true)
                                                                       │ */response_pwm
   REAL NUWave nodes above, SIMULATOR below                            ▼
                                                    sim_motor_interface  (duty → µs → N, or rad/s)
                                                                       │ /sim/*/cmd
                     ros_gz_bridge  ◀──────────────────────────────────┘
                          │  ▲
                          ▼  │            imu_sim ──▶ /imu  (gravity removed, imu_link)
                       Gazebo ───────────▶ depth_sensor_sim ──▶ /depth/{depth,depth_array,pressure,temperature}
                                          camera_sim ──▶ /camera_{0,1}/image/compressed (JPEG, best effort)
```

## Quick start (ROS 2 Humble, Ubuntu 22.04 / WSL)

    sudo apt install ros-humble-ros-gzharmonic ros-humble-joy python3-opencv   # Gazebo Harmonic + bridge
    cd ~/nuwave_ws/nuwave-rov && colcon build --symlink-install && source install/setup.bash
    ros2 launch mate_rov_sim sim.launch.py

Plug in the same two controllers you use on the real ROV (PowerA Xbox Series X for
driving, Xbox 360 for the arm) and drive. On WSL, attach them to Linux first with
[usbipd](https://learn.microsoft.com/windows/wsl/connect-usb). Check the names SDL sees with
`ros2 run joy joy_enumerate_devices`, then pass them if they differ:

    ros2 launch mate_rov_sim sim.launch.py joy_thruster_device:="<name>" joy_arm_device:="<name>"
    ros2 launch mate_rov_sim sim.launch.py joy_thruster_device:="" joy_thruster_id:=0   # pick by index

Everything the pilot normally does works the same: precision mode, expo, stabilize
toggle, roll buttons, arm sticks/triggers/bumpers.

### Keyboard

No gamepad? A small **"ROV keyboard teleop"** window opens with the sim. Click it
(keys only register while it has focus) and hold keys to drive:

| ROV | | Arm | |
|---|---|---|---|
| W / S | forward / back | J / L | base yaw left / right |
| A / D | strafe left / right | I / K | shoulder up / down |
| Q / E | yaw left / right | U / O | elbow up / down |
| R / F | up / down | N / M | wrist rotate |
| ↑ / ↓ | pitch nose down / up | Y / H | wrist pitch up / down |
| ← / → | roll left / right | [ / ] | claw open / close |

P = precision mode, X = stabilize, Z = expo (toggles, like the gamepad buttons).

The keyboard doesn't bypass anything: it publishes gamepad-style `Joy` messages built
from houston's `joystick_config.yaml`, so keys go through houston exactly like the
controllers. `joy_mux` merges both sources: **while any key is held the keyboard
wins; otherwise the gamepad passes through untouched**, and with neither active
houston gets neutral (stop). Use both at once, e.g. drive with the gamepad and tap
keys for the arm. Launch with `keyboard:=false` to skip the window (headless, SSH).

### Launch arguments

| argument | default | what it does |
|---|---|---|
| `topside` | `true` | joy nodes, joy_mux, houston, thruster controller, arm controller |
| `keyboard` | `true` | keyboard teleop window (needs a display) |
| `use_pwm_node` | `true` | real `pwm_node simulate:=true` between controllers and sim (slew limit, watchdog, precision ranges). `false` = sim reads the controllers' duty cycles directly |
| `stabilization` | `true` | `stabilization_node` |
| `web_gui` | `false` | `web_gui/bridge_node` → http://localhost:8080 |
| `render_engine` | `ogre` on WSL, else `ogre2` | Gazebo renderer for the GUI and cameras. On WSL, ogre2 renders ~5x slower and drags the whole sim far below real time |
| `gz_args` | `-r -v 3` | extra `gz sim` flags; add `-s` for headless |
| `joy_thruster_device` / `joy_arm_device` | real controller names | SDL device names; empty = use `joy_thruster_id` / `joy_arm_id` |

Expect the sim to run somewhat slower than real time on a laptop (about 0.5–0.6x
with both cameras on WSL). Everything stays consistent, it just feels like slow motion.
Check with `gz topic -e -t /stats | grep real_time_factor`.

## What is simulated

* **Vehicle frame = NUWave control frame:** x right, y forward, z up, origin at the
  center of mass. The same frame `controller/config/thruster_config.yaml`, houston and
  the IMU use, so no conversions anywhere.
* **8 thrusters** at exactly the positions and directions in
  `controller/config/thruster_config.yaml`. Thrust comes from the pulse width pwm_node
  would send: the T200 table in `config/rov_params.yaml` (`pwm_points`/`thrust_points`),
  shifted so each thruster stops at its own `neutral_us` from
  `thruster_pkg/config/individual_motor_config.yaml`. Precision mode changes the
  range exactly as on the robot.
* **Arm:** a placeholder 5-DOF arm + claw made of simple shapes, one joint per motor
  in `arm_controller/config/arm_config.yaml`. The real arm motors are speed-controlled,
  so duty cycle → joint speed (`arm_max_speed`, `claw_max_speed`). It's neutrally
  buoyant so it doesn't change trim. There's a sinking pipe in front of the ROV to
  practice grabbing.
* **Sensors**, on the same topics/types as the real drivers:
  `/imu` (BNO08x-style: gravity removed, relative yaw, `imu_link`),
  `/depth/*` (MS5837-style, 10 Hz, positive down),
  `/camera_0` (forward) and `/camera_1` (claw cam), JPEG 320x240.
* **Physics:** buoyancy (water below z = 0, air above), drag, added mass. Mass,
  buoyancy, drag and thrust limits are still the **starter placeholders** for a
  ~12 kg T200 ROV. Replace them with measurements of the real vehicle.

Simulator internals (only the sim nodes use these): `/sim/thruster_*_joint/cmd` (N),
`/sim/arm_*_joint/cmd` (rad/s), `/sim/imu_raw`, `/sim/camera_N/image`,
`/sim/odom_ground_truth` (perfect pose), `/sim/joint_states` (arm angles).
Push the ROV with a current: `ros2 topic pub --once /ocean_current geometry_msgs/msg/Vector3 "{x: 0.2}"`.

## Keeping it in sync with the robot

The model duplicates some facts from the NUWave configs. If you change thruster
geometry or add/rename motors, update `models/mate_rov/model.sdf` and
`config/bridge.yaml` to match, then run the consistency tests:

    cd src/mate_rov_sim && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test

Naming rule: motor `/thruster/<name>` ↔ joint `<name>_joint`; motor `/arm/motor_<name>` ↔
joint `arm_<name>_joint`.

Check trim after changing mass or collisions:

    python3 tools/buoyancy_check.py models/mate_rov/model.sdf --frame 0.45 0.35 0.28

## Using the real CAD

Gazebo can't read SolidWorks files directly. Export each rigid part as STL or
(better, keeps colors) COLLADA `.dae` (or use the SolidWorks-to-URDF exporter, which
also gives masses and inertias), put the meshes in `models/mate_rov/meshes/`, and
reference them from the `<visual>` elements:

    <geometry><mesh><uri>meshes/frame.dae</uri><scale>0.001 0.001 0.001</scale></mesh></geometry>

(scale 0.001 if exported in millimeters). Keep the `<collision>` shapes as simple boxes
and cylinders: buoyancy is computed from them and Gazebo ignores mesh collisions for
buoyancy. `src/arm_simulation/urdf/armsim.urdf.xacro` is a SolidWorks export of the arm,
but its STL meshes aren't in the repository and its masses (~18 kg total) look like
SolidWorks default densities, so the sim uses the placeholder arm for now.

## Layout

    launch/sim.launch.py                    Gazebo + bridge + sim stand-ins + real NUWave nodes
    worlds/mate_pool.sdf                    pool (8 x 12 x 4 m), buoyancy, props, includes the ROV
    models/mate_rov/model.sdf               vehicle: mass, buoyancy, 8 thrusters, arm, drag, sensors
    config/bridge.yaml                      Gazebo <-> ROS 2 topic mapping (/sim/...)
    config/rov_params.yaml                  sim stand-in parameters (thrust table, arm speeds, sensors)
    mate_rov_sim/sim_motor_interface.py     stands in for PCA9685 + ESCs + motors
    mate_rov_sim/imu_sim.py                 stands in for imu_pkg/imu_pub
    mate_rov_sim/depth_sensor_sim.py        stands in for rov_depth_sensor
    mate_rov_sim/camera_sim.py              stands in for camera_pkg
    mate_rov_sim/keyboard_teleop.py         keyboard -> gamepad-style Joy (window)
    mate_rov_sim/joy_mux.py                 keyboard / gamepad arbitration in front of houston
    test/test_model_matches_config.py       model / bridge / NUWave config consistency checks
    test/test_keyboard_mapping.py           every key through houston's real parser and config
    tools/buoyancy_check.py                 mass / buoyancy / CoB / drag calculator
