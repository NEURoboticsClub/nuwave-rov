# Source - https://stackoverflow.com/a
# Posted by Bharath Kumar
# Retrieved 2025-11-12, License - CC BY-SA 4.0

#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
    
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.declare_parameter('camera_address', 0)
        camera_source = self.get_parameter('camera_address').get_parameter_value().integer_value

        topic_name='video_'+ str(camera_source)

        self.subscription = self.create_subscription(Image, topic_name, self.img_callback, 10)
        self.subscription 
        self.br = CvBridge()

    def img_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)   
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()
