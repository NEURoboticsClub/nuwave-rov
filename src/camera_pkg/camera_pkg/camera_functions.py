import numpy as np
import cv2
import threading
import time
from sensor_msgs.msg import CompressedImage


def setup_camera(device_path, width, height, fps, get_logger, cap=None):
    while True:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                get_logger.warning("Could not release video capture despite capture object != None.")
            cap = None
        try:
            cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
            if cap.isOpened():
                break
        except Exception as e:
            get_logger.warning(f"Failed to open camera {device_path}: {e}")
        time.sleep(0.1)
 
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def capture_loop(cap, reopen_camera, on_frame, running, get_clock, get_logger):
    """Continuously grab frames, overwrite old ones"""
    fail_count = 0
    while running:
        try:
            ret, frame = cap.read()
        except Exception as e:
            get_logger().warning(f"Camera read raised: {e}")
            ret, frame = False, None
        if not ret:
            fail_count += 1
            if (fail_count > 10):
                cap = reopen_camera(cap)
                fail_count = 0
                continue
            time.sleep(0.01)
            continue
        fail_count = 0

        stamp = get_clock().now().to_msg()
        on_frame(frame, stamp)


def publish_frame(frame, stamp, publisher, frame_id, jpeg_quality, get_logger):
    try:
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        success, encoded = cv2.imencode('.jpg', frame, encode_params)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()

        publisher.publish(msg)
    except Exception as e:
        get_logger.warning(f"Failed to encode/publish frame: {e}")
