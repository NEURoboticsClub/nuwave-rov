import os

from ament_index_python.packages import get_package_share_directory  # type: ignore[reportMissingImports]
from nuwave_utils_pkg.file_helpers import load_yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

from arm_controller_pkg.arm_controller_functions import status_callback, publish_arm_motors



class ArmController(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('arm_controller')

        # Parameters
        self.declare_parameter('publish_rate_hz', 50.0)

        pkg_share = get_package_share_directory('arm_controller_pkg')
        self.declare_parameter(
            'arm_config',
            os.path.join(pkg_share, 'config', 'arm_config.yaml')
        )
        self.declare_parameter('arm_topic', 'arm_commands')

        rate = float(self.get_parameter('publish_rate_hz').value)

        arm_config_path = self.get_parameter('arm_config').value
        arm_topic = self.get_parameter('arm_topic').value

        self.get_logger().info(f'Loading arm config from: {arm_config_path}')

        config = load_yaml(arm_config_path)
        # self.pwm_range_table = self._configure_arm_motor_pwm_range(config)
        self.motors = config.get('arm_motors', [])

        # Subscribers / Publishers
        self.status_sub = self.create_subscription(
            Float32MultiArray, arm_topic, self.Status_Callback, 10)
        self.arm_motor_pubs = []
        for mot in self.motors:
            pub = self.create_publisher(Float32, mot.get('topic'), 10)
            self.arm_motor_pubs.append(pub)
            mot['pub'] = pub
            mot['dutycycle'] = 0

        # Init command send neutral to all arm motors
        # self.pwm_commands = self.pwm_range_table[:,1]

        # Watchdog: zero outputs if no command received within timeout
        self.declare_parameter('watchdog_timeout_s', 0.5)
        self.watchdog_timeout_s = float(self.get_parameter('watchdog_timeout_s').value)
        self.last_msg_time = None
        self.watchdog_tripped = False

        self.create_timer(1.0 / rate, self.Publish_Arm_Motors)
        self.get_logger().info('Arm Controller Initialized')

    def Status_Callback(self, msg: Float32MultiArray):
        """Process a new velocity vector and turns them into PWM signals for the Arm Motors."""
        self.last_state_msg = msg
        self.last_msg_time = status_callback(
            msg, self.motors, self.get_logger(), self.get_clock())

    def Publish_Arm_Motors(self):
        """Publish current PWM commands to each arm motor."""
        self.watchdog_tripped = publish_arm_motors(
            self.motors,
            self.get_clock(),
            self.get_logger(),
            self.watchdog_timeout_s,
            self.last_msg_time,
            self.watchdog_tripped,
        )


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
