import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs import msg
from std_msgs.msg import Float32, Float32MultiArray
from ament_index_python.packages import get_package_share_directory
from nuwave_utils_pkg.file_helpers import load_yaml
import os
import numpy as np

class ArmController(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('arm_controller')

        # Parameters
        self.declare_parameter('publish_rate_hz', 50.0)

        pkg_share = get_package_share_directory('arm_controller')
        self.declare_parameter(
                'arm_config', 
                os.path.join(pkg_share, 'config', 'arm_config.yaml')
                )
        self.declare_parameter('arm_topic', 'arm_commands')

        rate = float(self.get_parameter('publish_rate_hz').value)

        arm_config_path = self.get_parameter('arm_config').value
        arm_topic = self.get_parameter('arm_topic').value

        self.get_logger().info(f"Loading arm config from: {arm_config_path}")

        config = load_yaml(arm_config_path)
        # self.pwm_range_table = self._configure_arm_motor_pwm_range(config)
        self.motors = config.get('arm_motors', [])

        # Subscribers / Publishers
        self.status_sub = self.create_subscription(Float32MultiArray, arm_topic, self.Status_Callback, 10)
        self.arm_motor_pubs = [] 
        for mot in self.motors:
            pub = self.create_publisher(Float32, mot.get('topic'), 10)
            self.arm_motor_pubs.append(pub)
            mot['dutycycle'] = 0
        
        # Init command send neutral to all arm motors
        # self.pwm_commands = self.pwm_range_table[:,1]

        # Watchdog: zero outputs if no command received within timeout
        self.declare_parameter('watchdog_timeout_s', 0.5)
        self.watchdog_timeout_s = float(self.get_parameter('watchdog_timeout_s').value)
        self.last_msg_time = None

        self.create_timer(1.0 / rate, self.publish_arm_motors)
        self.get_logger().info("Arm Controller Initialized")
        
    def Status_Callback(self, msg: Float32MultiArray):
        """
        Process a new velocity vector and turns them into PWM signals for the Arm Motors.
        """
        self.last_state_msg = msg
        self.last_msg_time = self.get_clock().now()

        arm_velocities = np.array(msg.data, dtype=float)

        for mot in self.motors:
            mot['dutycycle'] = arm_velocities[mot['id'] - 1]

    def publish_arm_motors(self):
        """
        Publish current PWM commands to each arm motor
        """
        timed_out = (
            self.last_msg_time is None or
            (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9 > self.watchdog_timeout_s
        )
        for mot in self.motors:
            msg = Float32()
            msg.data = 0.0 if timed_out else float(mot.get('dutycycle', 0.0))
            self.arm_motor_pubs[mot['id'] - 1].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
