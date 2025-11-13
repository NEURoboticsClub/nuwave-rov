import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ThrustTester(Node):
    def __init__(self):
        super().__init__('thrust_tester')
        self.pub = self.create_publisher(Float32, '/cmd_thrust', 10)

        # publish every 0.05 s ≈ 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.t0 = time.time()
        self.get_logger().info("ThrustTester started – sweeping between −1 and 1")

    def timer_callback(self):
        # create a smooth sine-wave sweep
        t = time.time() - self.t0
        cmd = math.sin(2 * math.pi * 0.1 * t)  # 0.1 Hz = 10 s full period
        msg = Float32()
        msg.data = cmd
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing thrust {cmd:+.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ThrustTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
