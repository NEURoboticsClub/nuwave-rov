#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
import cv2
import threading
import time


class FastCameraPublisher(Node):
    def __init__(self):
        super().__init__('fast_camera_publisher')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 70)

        cam_id = self.get_parameter('camera_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        topic = f'/camera_{cam_id}/image/compressed'

        # ---------------- QoS (LOW LATENCY) ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(
            CompressedImage,
            topic,
            qos
        )

        # ---------------- CAMERA SETUP ----------------
        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {cam_id}")

        # Force MJPEG (HUGE performance win)
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # ---------------- THREADING ----------------
        self.frame = None
        self.lock = threading.Lock()
        self.running = True

        self.capture_thread = threading.Thread(
            target=self.capture_loop,
            daemon=True
        )
        self.capture_thread.start()

        self.timer = self.create_timer(
            1.0 / fps,
            self.publish_frame
        )

        self.get_logger().info(
            f"Camera {cam_id} streaming on {topic} "
            f"({width}x{height} @ {fps} FPS)"
        )

    # ------------------------------------------------

    def capture_loop(self):
        """Continuously grab frames, overwrite old ones"""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            with self.lock:
                self.frame = frame

    def publish_frame(self):
        with self.lock:
            if self.frame is None:
                return
            frame = self.frame.copy()

        encode_params = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            self.jpeg_quality
        ]
        success, encoded = cv2.imencode('.jpg', frame, encode_params)
        if not success:
            return

        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()

        self.publisher.publish(msg)

    # ------------------------------------------------

    def destroy_node(self):
        self.running = False
        self.capture_thread.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FastCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# Source - https://stackoverflow.com/a
# Posted by Bharath Kumar
# Retrieved 2025-11-09, License - CC BY-SA 4.0

#! /usr/bin/env python3

# import rclpy
# from rclpy.node import Node 
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2 
    
# class CameraPublisher(Node):
#     def __init__(self):
#         super().__init__('camera_publisher')

#         self.declare_parameter('camera_address', 0)
#         camera_source = self.get_parameter('camera_address').get_parameter_value().integer_value

#         topic_name='video_'+ str(camera_source)

#         self.publisher_ = self.create_publisher(Image, topic_name , 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)

        
#         self.cap = cv2.VideoCapture(camera_source)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
#         self.br = CvBridge()

#     def timer_callback(self):
#         ret, frame = self.cap.read()     
#         if ret == True:
#             self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
#         self.get_logger().info('Publishing video frame')


# def main(args=None):
#     rclpy.init(args=args)
#     camera_publisher = CameraPublisher()
#     rclpy.spin(camera_publisher)
#     camera_publisher.destroy_node()
#     rclpy.shutdown()

  
# if __name__ == '__main__':
#   main()

# # Open the default camera
# cam = cv2.VideoCapture(8)
# cam2 = cv2.VideoCapture(0)

# # Get the default frame width and height
# frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
# frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# # Define the codec and create VideoWriter object
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))

# while True:
#     ret, frame = cam.read()
#     ret2, frame2 = cam2.read()

#     # Write the frame to the output file
#     out.write(frame)
#     out.write(frame2)

#     # Display the captured frame
#     cv2.imshow('Camera', frame)
#     cv2.imshow('Camera 2', frame2)

#     # Press 'q' to exit the loop
#     if cv2.waitKey(1) == ord('q'):
#         break

# # Release the capture and writer objects
# cam.release()
# cam2.release()
# out.release()
# cv2.destroyAllWindows()

