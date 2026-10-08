# Crab Recognition

## Purpose

The `crab_recognition_node` node runs a YOLO model against live camera frames to detect European green crabs. On trigger, it runs detection once per camera, draws bounding boxes and a per-camera count on the frame, and saves the annotated images to disk.

It subscribes to one or more compressed camera image topics and buffers the latest frame from each. When triggered via a boolean topic, it runs the model against the most recently received frame from every camera and writes the annotated results as separate image files.

## How to run

### Running with default configs

```
ros2 run crab_recognition_pkg crab_recognition
```

### Using a custom config

```
ros2 run crab_recognition_pkg crab_recognition \
  --ros-args \
  -p model_path:=/path/to/model.pt \
  -p output_path:=/path/to/result.jpg \
  -p confidence:=0.4 \
  -p camera_topic_prefix:=/camera_ \
  -p camera_count:=4
```

One or more of the above params are optional

## Subscribed Topics

| Topic                                               | Type                              | Description                                                               |
| --------------------------------------------------- | --------------------------------- | ------------------------------------------------------------------------- |
| `/gui_buttons/detect_crabs`                         | `std_msgs/msg/Bool`               | Trigger to run detection once across all buffered camera frames.          |
| `<camera_topic_prefix><camera_id>/image/compressed` | `sensor_msgs/msg/CompressedImage` | Live compressed image feed for each camera, buffered as the latest frame. |

## Published Topics

This node does not publish any topics. Detection results are written to disk instead (see Configuration).

## Parameters

| Parameter             | Default                                        | Description                                                              |
| --------------------- | ---------------------------------------------: | ------------------------------------------------------------------------ |
| `model_path`          | `.../crab_recognition_models/30epochs_best.pt` | Path to the YOLO model weights file.                                     |
| `output_path`         | `.../crab_recognition_results/result.jpg`      | Base path used to derive per-camera, timestamped output image filenames. |
| `confidence`          | `0.4`                                          | Minimum confidence threshold for YOLO detections.                        |
| `camera_topic_prefix` | `/camera_`                                     | Prefix used to build each camera's image topic name.                     |
| `camera_count`        | `4`                                            | Number of camera topics to subscribe to (clamped to 1–4).                |

## Configuration

Output images are named from `output_path` by inserting a run timestamp and camera id before the file extension, e.g. `result_20260927_141200_123456_camera0.jpg`. The parent directory of `output_path` is created automatically if it doesn't exist.

Each camera subscription uses a best-effort, keep-last-1 QoS profile suited to high-rate image streams.

## Methods

| Method                 | Description                                                                                                 |
| ---------------------- | ----------------------------------------------------------------------------------------------------------- |
| `__init__()`           | Initializes the node, parameters, YOLO model, camera subscriptions, and the detection trigger subscription. |
| `crab_scan_callback()` | Handles the detection trigger, guarding against re-entrant runs while a detection is in progress.           |
| `camera_callback()`    | Decodes an incoming compressed image and stores it as the latest frame for that camera.                     |
| `run_detection_once()` | Runs YOLO on the latest frame from each camera, draws boxes/labels/count, and saves annotated images.       |
| `main()`               | Initializes ROS 2, starts the node, and runs the ROS 2 event loop.                                          |

## Dependencies

### ROS 2 Packages

* `rclpy`
* `std_msgs`
* `sensor_msgs`

### Python Packages

* `opencv-python` (`cv2`)
* `numpy`
* `ultralytics`

## External Documentation

* [sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)
* [std_msgs/msg/Bool](https://docs.ros.org/en/rolling/p/std_msgs/msg/Bool.html)
* [Ultralytics YOLO](https://docs.ultralytics.com/)
