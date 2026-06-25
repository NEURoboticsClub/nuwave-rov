"""Mock camera node that publishes test JPEG video on all 4 camera topics.

Matches the real camera output (sensor_msgs/CompressedImage, format='jpeg')
on /camera_{0..3}/image/compressed, so the web GUI and screenshot button
work with no real cameras attached.

    python3 src/camera_pkg/camera_pkg/mock_cameras.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class MockCameras(Node):
    def __init__(self, num_cameras=4, width=640, height=480, fps=30.0):
        super().__init__('mock_cameras')
        self.width = width
        self.height = height
        self.frame = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publishers_ = [
            self.create_publisher(
                CompressedImage, f'/camera_{i}/image/compressed', qos)
            for i in range(num_cameras)
        ]
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(
            f'Publishing random video on {num_cameras} cameras at {fps} fps')

    def tick(self):
        self.frame += 1
        for cam_id, pub in enumerate(self.publishers_):
            # Generates compressible animated content
            shade = (self.frame * 3 + cam_id * 40) % 256
            img = np.empty((self.height, self.width, 3), dtype=np.uint8)
            img[:, :, 0] = shade
            img[:, :, 1] = (255 - shade)
            img[:, :, 2] = (shade // 2 + cam_id * 30) % 256
            cx = int((self.frame * 6) % self.width)
            cy = self.height // 2 + int(60 * np.sin(self.frame / 10.0))
            cv2.circle(img, (cx, cy), 50, (255, 255, 255), -1)
            cv2.putText(img, f'MOCK CAM {cam_id}  frame {self.frame}',
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 0), 2)
            ok, encoded = cv2.imencode('.jpg', img)
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'camera_{cam_id}'
            msg.format = 'jpeg'
            msg.data = encoded.tobytes()
            pub.publish(msg)


def main():
    rclpy.init()
    node = MockCameras()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
