#!/usr/bin/env python3
"""
camera_sim: the simulator's stand-in for camera_pkg/camera_publisher.

Takes the raw images Gazebo renders and republishes them exactly the way
camera_pkg does: JPEG-compressed, BEST_EFFORT, depth 1, so the web GUI and
crab_recognition work unchanged.

Subscribes:  /sim/camera_<id>/image             sensor_msgs/Image (rgb8, from Gazebo)
Publishes:   /camera_<id>/image/compressed      sensor_msgs/CompressedImage, frame camera_<id>
"""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraSim(Node):
    def __init__(self):
        super().__init__('camera_sim')
        self.declare_parameter('camera_ids', [0, 1])
        self.declare_parameter('input_topic_format', '/sim/camera_{}/image')
        self.declare_parameter('output_topic_format', '/camera_{}/image/compressed')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('jpeg_quality', 70)

        gp = self.get_parameter
        self.jpeg_quality = int(gp('jpeg_quality').value)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.latest = {}
        self.pubs = {}
        for cam_id in gp('camera_ids').value:
            self.latest[cam_id] = None
            self.pubs[cam_id] = self.create_publisher(
                CompressedImage, gp('output_topic_format').value.format(cam_id), qos)
            self.create_subscription(Image, gp('input_topic_format').value.format(cam_id),
                                     lambda msg, c=cam_id: self.latest.__setitem__(c, msg),
                                     qos_profile_sensor_data)
        self.create_timer(1.0 / gp('fps').value, self.on_timer)

    def on_timer(self):
        for cam_id, img in self.latest.items():
            if img is None:
                continue
            self.latest[cam_id] = None  # publish each rendered frame once
            if img.encoding not in ('rgb8', 'bgr8'):
                self.get_logger().warn(f'camera {cam_id}: unsupported encoding {img.encoding}',
                                       throttle_duration_sec=5.0)
                continue
            frame = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.step)
            frame = frame[:, :img.width * 3].reshape(img.height, img.width, 3)
            if img.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # wall time, like camera_pkg
            msg.header.frame_id = f'camera_{cam_id}'
            msg.format = 'jpeg'
            msg.data = encoded.tobytes()
            self.pubs[cam_id].publish(msg)


def main():
    rclpy.init()
    node = CameraSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
