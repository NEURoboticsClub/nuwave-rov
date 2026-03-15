import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from collections import deque
import numpy as np


class AutoPitchNode(Node):
    """The auto pitch node minimizes change in pitch using IMU measurements and bang-bang control.
    This node assumes auto pitch is always activated."""

    def __init__(self):
        self.pitch_history = deque(maxlen=10)
        self.bang_bang_radius = 0.05

        super().__init__("auto_pitch_node")
        self.get_logger().info("Created auto pitch node")

        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.pitch_control_pub = self.create_publisher(Twist, "/pitch_command", 10)  ## to status node

    
    def imu_callback(self, msg):
        """Handles received imu measurement by extracting pitch"""
        self.pitch_history.append(msg.angular_velocity.y)

        y_velocity = self.pure_bang_bang()

        pitch_command = Twist()
        pitch_command.angular.y = y_velocity
        self.pitch_control_pub.publish(pitch_command)
        self.get_logger().info(f"Published y velocity = {y_velocity}")


    def pure_bang_bang(self) -> float:
        """Computes y velocity based on history"""
        current_pitch = np.mean(list(self.pitch_history))  ## Use last 10 pitch readings to handle noise

        if current_pitch > self.bang_bang_radius:
            return -0.1
        elif current_pitch < -self.bang_bang_radius:
            return 0.1
        else:
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = AutoPitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
