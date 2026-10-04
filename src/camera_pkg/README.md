# Camera (camera_pkg)
This package drives the ROV's underwater cameras, capturing from each camera's V4L2 device and publishing compressed video so the topside GUI can display it. The ROV uses DeepWater Exploration exploreHD USB cameras, which are UVC-compliant and show up as standard `/dev/videoN` devices, so frames are grabbed directly with OpenCV (no vendor SDK needed).

A background thread continuously grabs frames while a separate timer encodes and publishes the latest one at the configured FPS, so a slow camera read never backs up the publish rate. If a camera stops responding, the node reopens the device automatically.

## Requirements
Requires `rclpy`, `sensor_msgs`, `cv_bridge`, and OpenCV (`cv2`), along with `cv2_enumerate_cameras` (used for USB camera auto-discovery) and `numpy` (used by the mock camera node).

## How to run
### Running with default configs (auto-discovers connected cameras)
```
ros2 launch camera_pkg multi_camera_launch.launch.py
```
### Running a single camera manually
```
ros2 run camera_pkg camera_publisher \
  --ros-args \
  -p camera_id:=0 \
  -p camera_device_path:=/dev/video0
```
### Running without hardware (mock cameras)
```
python3 src/camera_pkg/camera_pkg/mock_cameras.py
```
Publishes synthetic animated frames on the same topics real cameras use, so the GUI can be tested with no cameras attached.

## Topics
### Publishing
- sensor_msgs/CompressedImage data (jpeg) on topic /camera_\<camera_id\>/image/compressed, one topic per connected camera

### Subscribing
- camera_publisher does not subscribe to anything
- (dev-only) camera_subscriber subscribes to sensor_msgs/Image on topic video_\<camera_address\>, for viewing a raw feed with OpenCV during debugging

## Configs
- camera_id
    - Parameter, int; logical camera index, used to build the topic name and frame_id
- camera_device_path
    - Parameter, string; V4L2 device path, e.g. /dev/video0. Set per-camera by the launch file
- width / height
    - Parameters, int; capture resolution, default 320x240
- fps
    - Parameter, int; capture and publish rate, default 30
- jpeg_quality
    - Parameter, int 0-100; JPEG encode quality, default 70

An alternate GStreamer-based streaming path (`gscam_launch.py`) also exists using the external `gscam2` package instead of this package's own publisher. It isn't used by default.

## External Docs
- [exploreHD product page](https://dwe.ai/products/explorehd) — the camera hardware used on the ROV
- [gscam2](https://github.com/clydemcqueen/gscam2) — ROS 2 driver used by the alternate GStreamer launch path
