import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion, Twist
from nuwave_utils_pkg.file_helpers import load_yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty

from stabilization_pkg import stabilization_functions


class StabilizationNode(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('stabilization')

        # Parameters
        self.declare_parameter('publish_rate_hz', 50.0)

        pkg_share = get_package_share_directory('stabilization_pkg')
        self.declare_parameter(
            'stabilization_config',
            os.path.join(pkg_share, 'config', 'stabilization_config.yaml')
        )

        stabilization_config_path = self.get_parameter('stabilization_config').value

        self.get_logger().info(f'Loading stabilization config from: {stabilization_config_path}')

        config = load_yaml(stabilization_config_path)
        if not isinstance(config, dict):
            self.get_logger().error(
                'Stabilization config is not a mapping; falling back to defaults'
            )
            config = {}

        self.imu_stabilization_params = config.get('imu_stabilization', {}) or {}
        self.depth_stabilization_params = config.get('depth_stabilization', {}) or {}

        # Subscribers / Publishers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.capture_sub = self.create_subscription(
            Empty, '/stabilizer/capture', self.capture_callback, 10)
        self.last_imu_orientation = None

        self.sta_pub = self.create_publisher(Twist, '/stabilizer/commands', 10)

        self.desired_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info('Stabilization Node Initialized')

    @staticmethod
    def _quat_mul(q1, q2):
        return stabilization_functions._quat_mul(q1, q2)

    def set_desired_orientation(self, q: Quaternion) -> bool:
         
        state = {'desired_orientation': self.desired_orientation} #Just a wrapper around the desired orientation quaternion to keep its state
        success = stabilization_functions.set_desired_orientation(
            q, state, self.get_logger()
        )
        if success:
            self.desired_orientation = state['desired_orientation']
        return success

    def _imu_to_twist_control(self, imu_msg: Imu) -> Twist:
        return stabilization_functions._imu_to_twist_control(
            imu_msg,
            self.desired_orientation,
            self.imu_stabilization_params,
            self.get_logger(),
        )

    def imu_callback(self, msg: Imu):
        """Handle new IMU data and publish a stabilization command."""
        self.last_imu_orientation = msg.orientation
        state = { #A wrapper around the IMU and desired orientation to keep their state
            'last_imu_orientation': self.last_imu_orientation,
            'desired_orientation': self.desired_orientation,
        }
        stabilization_functions.imu_callback(
            msg,
            state,
            self.imu_stabilization_params,
            self.get_logger(),
            self.sta_pub,
        )

    def capture_callback(self, msg: Empty):
        state = { #A wrapper around the IMU and desired orientation to keep their state
            'last_imu_orientation': self.last_imu_orientation,
            'desired_orientation': self.desired_orientation,
        }
        stabilization_functions.capture_callback(
            msg,
            state,
            self.imu_stabilization_params,
            self.get_logger(),
        )
        self.desired_orientation = state['desired_orientation']

    def publish_stabilization_commands(self, msg: Twist):
        """Publish new Twist commands to houston."""
        stabilization_functions.publish_stabilization_commands(msg, self.sta_pub)


def main(args=None):
    rclpy.init(args=args)
    node = StabilizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
