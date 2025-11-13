# Source - https://stackoverflow.com/a
# Posted by Bharath Kumar
# Retrieved 2025-11-09, License - CC BY-SA 4.0

#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
    
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('camera_address', 0)
        camera_source = self.get_parameter('camera_address').get_parameter_value().integer_value

        topic_name='video_'+ str(camera_source)

        self.publisher_ = self.create_publisher(Image, topic_name , 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(camera_source)
        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()     
        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()

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

