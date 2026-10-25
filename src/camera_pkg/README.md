# Camera

## Purpose

The `camera` node captures video from one of the ROV's underwater cameras and publishes it as compressed JPEG frames so the topside GUI can display it. The ROV uses DeepWater Exploration exploreHD USB cameras, which are UVC-compliant and show up as standard `/dev/videoN` devices, so frames are grabbed directly with OpenCV (no vendor SDK needed).

A background thread continuously grabs frames while a separate timer encodes and publishes the latest one at the configured FPS, so a slow camera read never backs up the publish rate. If more than 10 reads in a row fail, the node reopens the device automatically.

The launch file finds every connected exploreHD camera and starts one `camera` node for each, named `camera_<id>`.

## How to run
### Running all connected cameras (recommended)
```
ros2 launch camera_pkg multi_camera.launch.py
```

### Disabling a camera
```
ros2 launch camera_pkg multi_camera.launch.py cam1_enabled:=false
```

### Running a single camera
```
ros2 run camera_pkg camera_node \
  --ros-args \
  -p camera_id:=0 \
  -p camera_device_path:=/dev/video0
```
`camera_device_path` must be set. The node keeps retrying until the device opens, so it will not start if the path is wrong.

### Running without hardware
```
ros2 run camera_pkg mock_cameras_node
```
Publishes animated test frames on `/camera_0` to `/camera_3`, using the same topics and message type as the real cameras, so the GUI can be tested with no cameras attached.

## Subscribed Topics

This node doesn't subscribe to any topics.

## Published Topics

| Topic                                  | Type                              | Description                                                        |
| -------------------------------------- | --------------------------------- | ------------------------------------------------------------------ |
| `/camera_<camera_id>/image/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG frames from the camera, with `frame_id` `camera_<camera_id>`. |

The topic uses best-effort QoS with a depth of 1 for low latency. Subscribers must also use best-effort QoS, or they will not receive frames.

## Parameters

| Parameter            | Default | Description                                                             |
| -------------------- | ------: | ----------------------------------------------------------------------- |
| `camera_id`          |     `0` | Camera index, used in the topic name and `frame_id`.                    |
| `camera_device_path` |    `''` | V4L2 device path, e.g. `/dev/video0`. Required. Set by the launch file. |
| `width`              |   `320` | Capture width in pixels.                                                |
| `height`             |   `240` | Capture height in pixels.                                               |
| `fps`                |    `30` | Capture and publish rate.                                               |
| `jpeg_quality`       |    `70` | JPEG encode quality, from `0` to `100`.                                 |

## Configuration

`multi_camera.launch.py` lists the USB video devices and keeps the exploreHD cameras (USB vendor ID `3141`, product ID `25446`). It opens each one and starts a node only for cameras that return a frame, numbering them from `0` in the order found. Each node is started with `width` `320`, `height` `240`, `fps` `30`, and `jpeg_quality` `70`.

| Launch argument   | Default | Description                                  |
| ----------------- | ------: | -------------------------------------------- |
| `cam<N>_enabled`  |  `true` | Set to `false` to skip starting camera `N`.  |

If no working cameras are found, the launch file logs a message and exits.

## Methods

| Method            | Description                                                                                  |
| ----------------- | -------------------------------------------------------------------------------------------- |
| `__init__()`      | Sets up the parameters and publisher, opens the camera, and starts the capture thread and publish timer. |
| `setup_camera()`  | Opens the V4L2 device, retrying until it succeeds, and sets the MJPG format, resolution, and FPS. |
| `capture_loop()`  | Runs on the capture thread, keeping only the latest frame. Reopens the camera after more than 10 failed reads. |
| `publish_frame()` | Runs on the timer, encodes the latest frame as JPEG, and publishes it.                       |
| `destroy_node()`  | Stops the capture thread and releases the camera.                                            |
| `main()`          | Initializes ROS 2, starts the node, and runs the ROS 2 event loop.                           |

## Dependencies

### ROS 2 Packages

* `rclpy`
* `sensor_msgs`

### Python Packages

* `opencv-python` (`cv2`) — camera capture and JPEG encoding.
* `cv2_enumerate_cameras` — finds connected cameras in the launch file.
* `numpy` — generates test frames in the mock camera node.

### Project Packages

None.

## External Documentation

* [exploreHD product page](https://dwe.ai/products/explorehd) — the camera hardware used on the ROV
* [sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)
