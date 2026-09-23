# Camera Node (camera_pkg)

## Purpose

This package drives the ROV's underwater video cameras and gets their frames onto the ROS graph so the topside operator (via `web_gui`) can see what the vehicle sees.

The ROV is fitted with **DeepWater Exploration exploreHD** USB machine-vision cameras (UVC-compliant, USB Video Class). These present themselves to Linux as standard `/dev/videoN` V4L2 devices, so the package talks to them directly with OpenCV's `cv2.VideoCapture` (V4L2 backend) rather than requiring a vendor SDK.

Each physical camera gets its own `camera_publisher` node (`FastCameraPublisher`, in [`openCVStreaming.py`](camera_pkg/openCVStreaming.py)). Design goals:

- **Low latency over completeness.** Frame capture runs on a dedicated background thread that continuously grabs and overwrites the newest frame; a separate ROS timer, running at the configured `fps`, JPEG-encodes and publishes whatever the latest frame is. This decouples slow/blocking `cap.read()` calls from the publish rate and means the node never queues up stale frames.
- **Self-healing capture.** If 10 consecutive reads fail, the node tears down and reopens the V4L2 device (handles cameras that drop out or get replugged without requiring a node restart).
- **Bandwidth-conscious.** Frames are published pre-compressed as JPEG (`sensor_msgs/CompressedImage`) at a configurable resolution/quality, since the RQV video feed is piped over a tether/network link to the surface.
- **QoS tuned for live video.** `BEST_EFFORT` reliability with `KEEP_LAST` depth 1 — a dropped or late frame should be skipped, not retransmitted, so the feed stays real-time.

[`multi_camera_launch.launch.py`](launch/multi_camera_launch.launch.py) is the primary entry point: it auto-discovers every connected exploreHD camera by USB VID/PID (`0x0C45:0x6366`, the Sonix UVC controller exploreHD cameras use), verifies each candidate `/dev/videoN` path actually produces frames, and spawns one `camera_publisher` node per working camera (`camera_publisher_0`, `camera_publisher_1`, ...). This means camera indices/topics are stable based on discovery order even if `/dev/video*` numbering shifts between boots.

Two secondary tools live in this package for development without real hardware/on the bench:
- [`mock_cameras.py`](camera_pkg/mock_cameras.py) — publishes synthetic animated JPEG frames on the same 4 topics real cameras would use, so `web_gui` and the screenshot feature can be exercised with zero cameras attached.
- [`testCVStreaming.py`](camera_pkg/testCVStreaming.py) (`camera_subscriber`) — a debug viewer that opens an `cv2.imshow` window for a raw `Image` topic. **Note:** it subscribes to a `video_<camera_address>` topic using an uncompressed `sensor_msgs/Image`, which does not match the `/camera_<id>/image/compressed` topics the real publisher node emits — it's a legacy/standalone debugging script, not wired into the production pipeline.

An alternate GStreamer-based streaming path also exists ([`gscam_launch.py`](launch/gscam_launch.py) + [`config/gscam_params.yaml`](config/gscam_params.yaml)), which launches the external `gscam2` package instead of this package's own publisher node. It is not currently used by `multi_camera_launch.launch.py` and `gscam2` is not declared as a dependency in `package.xml`.

## Nodes / Executables

| Executable (`ros2 run camera_pkg ...`) | Source | Node name | Description |
|---|---|---|---|
| `camera_publisher` | `camera_pkg/openCVStreaming.py` (`FastCameraPublisher`) | `fast_camera_publisher` | Captures from one V4L2 camera and publishes compressed JPEG frames. One instance per physical camera. |
| `camera_subscriber` | `camera_pkg/testCVStreaming.py` (`CameraSubscriber`) | `camera_subscriber` | Dev-only viewer; subscribes to a raw `Image` topic and displays it with OpenCV. |

Not registered as console scripts (run directly with `python3`):

| Script | Description |
|---|---|
| `camera_pkg/mock_cameras.py` (`MockCameras`, node name `mock_cameras`) | Publishes synthetic video on `/camera_0..3` for hardware-free testing. Run with `python3 src/camera_pkg/camera_pkg/mock_cameras.py` from the repo root. |

## Topic Publishers

| Topic | Type | Published by | Notes |
|---|---|---|---|
| `/camera_<camera_id>/image/compressed` | `sensor_msgs/msg/CompressedImage` (`format: "jpeg"`) | `camera_publisher` (one node per camera, `camera_id` = 0, 1, 2, ...) | QoS: `BEST_EFFORT`, `KEEP_LAST`, depth 1. `header.frame_id` = `camera_<camera_id>`. Resolution/FPS/JPEG quality set via node parameters. |
| `/camera_0..3/image/compressed` | `sensor_msgs/msg/CompressedImage` | `mock_cameras` (dev/test tool) | Same topic naming/message shape as the real publisher so downstream nodes/GUI can't tell the difference. |

If `gscam_launch.py` is used instead, the external `gscam2` node (`gscam_publisher`) publishes its own standard image-transport topics (e.g. `image_raw`, `camera_info`, and compressed/theora variants if the corresponding `image_transport` plugins are installed) under its node namespace — these are not defined in this package.

## Topic Subscribers

| Topic | Type | Subscribed by | Notes |
|---|---|---|---|
| `video_<camera_address>` | `sensor_msgs/msg/Image` | `camera_subscriber` (`CameraSubscriber`) | `camera_address` is an integer node parameter (default `0`) used to build the topic name, e.g. `video_0`. Debug-only; does not match the topics the real `camera_publisher` node emits. |

The `camera_publisher` node (the production node) has no subscriptions — it only publishes.

Downstream, `web_gui`'s `bridge_node.py` subscribes to the `/camera_<id>/image/compressed` topics to forward frames to the topside web interface — see [`src/web_gui`](../web_gui).

## Node Parameters (`camera_publisher` / `FastCameraPublisher`)

| Parameter | Default | Description |
|---|---|---|
| `camera_id` | `0` | Logical camera index; used in the topic name and `frame_id`. |
| `camera_device_path` | `''` | V4L2 device path, e.g. `/dev/video0`. Set per-instance by the launch file. |
| `width` | `320` | Capture width. |
| `height` | `240` | Capture height. |
| `fps` | `30` | Capture/publish rate. |
| `jpeg_quality` | `70` | JPEG encode quality (0-100) passed to `cv2.imencode`. |

## Launch Files

- **`multi_camera_launch.launch.py`** — discovers connected exploreHD cameras via USB VID/PID + a V4L2 open/read check, then launches one `camera_publisher` node per working camera. Declares a `cam<N>_enabled` boolean launch arg per discovered camera. If no cameras are found (or discovery raises), it logs a message and starts no nodes rather than crashing the launch system.
- **`gscam_launch.py`** — launches the external `gscam2` package's `gscam_main` node using [`config/gscam_params.yaml`](config/gscam_params.yaml) (GStreamer `v4l2src` pipeline on `/dev/video0`). Alternate/legacy streaming path; not used by default.

## Dependencies

**ROS package dependencies** (declared in [`package.xml`](package.xml)):
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `cv2` (OpenCV)

**Python dependencies** (declared in [`setup.py`](setup.py)):
- `setuptools`
- [`cv2_enumerate_cameras`](https://pypi.org/project/cv2-enumerate-cameras/) — used by `multi_camera_launch.launch.py` to enumerate USB cameras by VID/PID.

**Implicit runtime dependencies** (imported in code but not declared in `package.xml`):
- `opencv-python` / `cv2` — video capture, JPEG encode/decode.
- `numpy` — used by `mock_cameras.py` to synthesize frames.

**Test dependencies:** `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

**Optional external ROS package:** [`gscam2`](https://github.com/clydemcqueen/gscam2) — only required if `gscam_launch.py` is used; not declared as a `package.xml` dependency.

## External Links / Documentation

- [DeepWater Exploration exploreHD product page](https://dwe.ai/products/explorehd) — camera hardware used by the ROV.
- [exploreHD product docs (DeepWater Exploration)](https://deepwaterexploration.github.io/legacydocs/products/explorehd.html)
- [Blue Robotics store listing — DeepWater Exploration exploreHD USB Camera](https://bluerobotics.com/store/sensors-cameras/cameras/deepwater-exploration-explorehd-usb-camera/)
- [`sensor_msgs/msg/CompressedImage` message definition](https://docs.ros2.org/latest/api/sensor_msgs/msg/CompressedImage.html)
- [`sensor_msgs/msg/Image` message definition](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)
- [OpenCV `cv2.VideoCapture` documentation](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)
- [Video4Linux2 (V4L2) API](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html)
- [`cv2_enumerate_cameras` on PyPI](https://pypi.org/project/cv2-enumerate-cameras/)
- [`gscam2` (ROS 2 GStreamer camera driver)](https://github.com/clydemcqueen/gscam2)
- [`cv_bridge` documentation](https://docs.ros.org/en/humble/p/cv_bridge/)

## Running

```bash
# Auto-discover connected exploreHD cameras and launch one publisher per camera
ros2 launch camera_pkg multi_camera_launch.launch.py

# Inspect a camera feed without hardware attached
python3 src/camera_pkg/camera_pkg/mock_cameras.py

# Manually run a single camera publisher
ros2 run camera_pkg camera_publisher --ros-args \
  -p camera_id:=0 \
  -p camera_device_path:=/dev/video0 \
  -p width:=320 -p height:=240 -p fps:=30 -p jpeg_quality:=70

# Inspect a published feed
ros2 topic hz /camera_0/image/compressed
```
