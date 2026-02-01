import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from collections import deque
import numpy as np


class AutoDepthNode(Node):
    def __init__(self):
        self.depth_history = deque(maxlen=10)
        self.target_depth = 0
        self.is_auto_depth_activated = False
        self.bang_bang_radius = 0.02
        self.p_factor = 1   ## for proportional bang bang

        super().__init__("auto_depth_node")
        self.get_logger().info("Created auto depth node")
        self.create_subscription(Float32, '/depth', self.depth_callback, 10)  ## from depth sensor node
        self.create_subscription(Bool, '/auto_depth_activated', self.auto_depth_activated_callback, 10)  ## from status node
        self.depth_control_pub = self.create_publisher(Twist, "/depth_command", 10) ## to status node

    def auto_depth_activated_callback(self, msg):
        """Handles command to toggle auto depth"""
        if msg.data and not self.is_auto_depth_activated:
            self.is_auto_depth_activated = True

            if len(self.depth_history) == 0:
                self.get_logger().error("Can't set target depth for auto depth control, no depth measurement history found")
                return
            self.target_depth = np.mean(self.depth_history)
            self.get_logger().info(f"Set auto depth target to {self.target_depth}")

        elif not msg.data and self.is_auto_depth_activated:
            self.is_auto_depth_activated = False
            self.get_logger().info("Deactivated auto depth")

        else:
            self.get_logger().info(f"Auto depth status kept as: {self.is_auto_depth_activated}")

    def depth_callback(self, msg):
        """Handles received depth measurement"""
        self.depth_history.append(msg.data)
        # self.get_logger().info(f"{self.depth_history}")

        if self.is_auto_depth_activated:
            z_velocity = self.pure_bang_bang()
            # z_velocity = self.proportional_bang_bang()

            depth_command = Twist()
            depth_command.linear.z = z_velocity
            self.depth_control_pub.publish(depth_command)
            # self.get_logger().info(f"Published {depth_command.linear.z}")

    def pure_bang_bang(self) -> float:
        current_depth = np.mean(list(self.depth_history)[-3:]) ## uses last 3 depth readings to handle noise

        if self.target_depth - current_depth > self.bang_bang_radius:
            return 1.0
        elif current_depth - self.target_depth > self.bang_bang_radius:
            return -1.0
        else:
            return 0.0
        
    def proportional_bang_bang(self) -> float:
        current_depth = np.mean(list(self.depth_history)[-3:]) ## uses last 3 depth readings to handle noise

        if abs(self.target_depth - current_depth) > self.bang_bang_radius:
            return (self.target_depth - current_depth) * self.p_factor
        else:
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = AutoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()