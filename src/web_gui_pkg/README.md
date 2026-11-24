# Web dashboard bridge

`web_bridge_node` connects ROS telemetry and compressed camera images to the
browser dashboard. It also publishes GUI mode changes and crab-detection
requests back to ROS, and saves camera screenshots and recording frames.

## Run

```sh
ros2 run web_gui_pkg web_bridge_node
```

Open `http://localhost:8080` on the machine running the node. The server binds to
localhost; accessing a remote instance requires suitable port forwarding.
`/?test` uses synthetic browser telemetry; `/?cams` opens the cameras-only view.

## ROS topics

| Direction | Topics | Message |
| --- | --- | --- |
| Subscribe | `/thruster/*`, `/arm/motor_*`, and their `/response_pwm` topics | `std_msgs/Float32` |
| Subscribe | `/power_monitor/monitor_0..7/{bus_voltage,current,power,shunt_voltage}` | `std_msgs/Float32` |
| Subscribe | Legacy `/power_monitor/{metric}` topics | `std_msgs/Float32`, mapped to monitor 0 |
| Subscribe | `/depth/depth`, `/depth/depth_array` | `Float32`, `Float32MultiArray` |
| Subscribe | `/depth/pressure`, `/depth/temperature`, `/imu` | `FluidPressure`, `Temperature`, `Imu` |
| Subscribe | `/velocity_commands`, `/arm_commands` | `Twist`, `Float32MultiArray` |
| Subscribe | `/controls/{expo_enabled,precision_mode,stabilize_enabled}` | `std_msgs/Bool` |
| Subscribe | `/camera_0..3/image/compressed` | `sensor_msgs/CompressedImage` |
| Publish | `/gui_buttons/detect_crabs` | `std_msgs/Bool` |
| Publish | `/gui_buttons/{expo_enabled,precision_mode,stabilize_enabled}` | `std_msgs/Bool` |

Braces and ranges above abbreviate the explicitly registered topics.

## Browser messages and files

`/ws` receives JSON objects shaped as `{"topic": "...", "data": ...}`. It sends
telemetry in the same envelope and video as one camera-ID byte followed by JPEG
bytes. Clients receive the latest control states when connecting. Buffers keep
the latest value per topic/frame so slow clients drop stale data.
Screenshot and recording controls are handled inside the bridge. Images go to
`~/rov_images` and writable removable mounts discovered under `/media/<user>`
and `/run/media/<user>`. Video recording writes PNG frames at 4 fps beneath
`rov_images/video/<session>/cam<id>`; it does not produce a video container file.
Disk writes run on background workers; the bounded recording queue can drop
frames when storage cannot keep up.

## Dependencies and configuration

Runtime imports require ROS 2 `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`,
`ament_index_python`, aiohttp, OpenCV, and NumPy. Camera/telemetry nodes must run
separately for live data. Current server/media settings are source constants
rather than ROS parameters.
See the [bridge source](web_gui_pkg/web_bridge_node.py),
[dashboard source](static/build_dashboard.js), and
[software setup](../../software_setup.md).
