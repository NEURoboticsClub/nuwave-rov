import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from datetime import datetime
from pathlib import Path
from ultralytics import YOLO


DEFAULT_MODEL_PATH = '/home/nuwave-rov/nuwave-rov/src/camera_pkg/camera_pkg/crab_recognition_models/30epochs_best.pt'
DEFAULT_OUTPUT_PATH = '/crab_recognition_results/result.jpg'
DEFAULT_CONFIDENCE = 0.4
DEFAULT_IMAGE_TOPIC_PREFIX = '/camera_'
DEFAULT_CAMERA_COUNT = 4


class CrabRecognitionNode(Node):
    def __init__(self):
        super().__init__('crab_recognition_node')

        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('output_path', DEFAULT_OUTPUT_PATH)
        self.declare_parameter('confidence', DEFAULT_CONFIDENCE)
        self.declare_parameter('camera_topic_prefix', DEFAULT_IMAGE_TOPIC_PREFIX)
        self.declare_parameter('camera_count', DEFAULT_CAMERA_COUNT)

        self.model_path = str(self.get_parameter('model_path').value)
        self.output_path = str(self.get_parameter('output_path').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.camera_topic_prefix = str(self.get_parameter('camera_topic_prefix').value)
        self.camera_count = max(1, min(4, int(self.get_parameter('camera_count').value)))
        self._busy = False
        self.latest_frames = {}
        self.camera_subs = []

        self.model = YOLO(self.model_path)
        self.get_logger().info(f'Loaded crab model from {self.model_path}')

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for camera_id in range(self.camera_count):
            topic = f'{self.camera_topic_prefix}{camera_id}/image/compressed'
            sub = self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, camera_id=camera_id: self.camera_callback(msg, camera_id),
                camera_qos,
            )
            self.camera_subs.append(sub)

        self.get_logger().info(
            f'Subscribed for crab detection on {self.camera_count} camera topics ({self.camera_topic_prefix}0..{self.camera_count - 1})'
        )

        self.crab_scan_sub = self.create_subscription(
            Bool,
            '/gui_buttons/detect_crabs',
            self.crab_scan_callback,
            10
        )

    def crab_scan_callback(self, msg: Bool):
        if not msg.data:
            return

        if self._busy:
            self.get_logger().warning('Crab detection already running; ignoring trigger.')
            return

        self._busy = True
        try:
            self.run_detection_once()
        except Exception as error:
            self.get_logger().error(f'Crab detection failed: {error}')
        finally:
            self._busy = False

    def camera_callback(self, msg: CompressedImage, camera_id: int):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.latest_frames[camera_id] = frame

    def run_detection_once(self):
        output_path = Path(self.output_path)
        if output_path.parent and str(output_path.parent) not in ('', '.'):
            output_path.parent.mkdir(parents=True, exist_ok=True)

        run_stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        output_suffix = output_path.suffix or '.jpg'

        if not self.latest_frames:
            self.get_logger().warning('No live frames received yet on camera topics; skipping detection trigger.')
            return

        total_count = 0
        for camera_id in sorted(self.latest_frames.keys()):
            img = self.latest_frames[camera_id].copy()
            results = self.model(img, conf=self.confidence, verbose=False)[0]

            green_count = 0
            for box in results.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id]

                if class_name != 'green_crab':
                    continue

                green_count += 1
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                cv2.rectangle(
                    img,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    img,
                    f'green_crab {conf:.2f}',
                    (x1 - 10, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_COMPLEX,
                    0.45,
                    (255, 0, 0),
                    1,
                    cv2.LINE_AA
                )

            cv2.putText(
                img,
                f'European Green Crabs: {green_count}',
                (10, 220),
                cv2.FONT_HERSHEY_COMPLEX,
                0.7,
                (0, 0, 0),
                1,
                cv2.LINE_AA
            )

            camera_output = output_path.with_name(
                f'{output_path.stem}_{run_stamp}_camera{camera_id}{output_suffix}'
            )
            cv2.imwrite(str(camera_output), img)
            total_count += green_count

            self.get_logger().info(
                f'Camera {camera_id} green crabs found: {green_count}; saved to {camera_output}'
            )

        self.get_logger().info(f'Total green crabs across {len(self.latest_frames)} camera frames: {total_count}')


def main(args=None):
    rclpy.init(args=args)
    crab_recognition_node = CrabRecognitionNode()
    try:
        rclpy.spin(crab_recognition_node)
    except KeyboardInterrupt:
        pass
    finally:
        crab_recognition_node.destroy_node()
        rclpy.shutdown()