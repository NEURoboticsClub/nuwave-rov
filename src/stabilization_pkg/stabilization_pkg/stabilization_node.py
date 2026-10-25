import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion, Twist
from nuwave_utils_pkg.file_helpers import load_yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from stabilization_pkg import stabilization_functions
from std_msgs.msg import Empty, Float32


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
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('depth_timeout_s', 0.5)

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

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn('publish_rate_hz must be positive; using 50 Hz')
            self.publish_rate_hz = 50.0
        self.imu_timeout_s = float(self.get_parameter('imu_timeout_s').value)
        self.depth_timeout_s = float(self.get_parameter('depth_timeout_s').value)
        self.velocity_filter_alpha = float(
            self.depth_stabilization_params.get('velocity_filter_alpha', 0.3)
        )

        self.desired_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.last_imu_orientation = None
        self.stabilization_state = {
            'desired_orientation': self.desired_orientation,
            'desired_depth': None,
            'last_imu_orientation': None,
            'last_imu_msg': None,
            'imu_sample_time': None,
            'current_depth': None,
            'depth_rate': 0.0,
            'depth_sample_time': None,
        }

        # Subscribers / Publishers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.depth_sub = self.create_subscription(
            Float32, '/depth/depth', self.depth_callback, 10
        )

        self.capture_sub = self.create_subscription(
            Empty, '/stabilizer/capture', self.capture_callback, 10)

        self.sta_pub = self.create_publisher(Twist, '/stabilizer/commands', 10)

        self.control_timer = self.create_timer(
            1.0 / self.publish_rate_hz, self.control_callback
        )

        self.get_logger().info('Stabilization Node Initialized')

    @staticmethod
    def _quat_mul(q1, q2):
        return stabilization_functions._quat_mul(q1, q2)

    def set_desired_orientation(self, q: Quaternion) -> bool:
        state = self.stabilization_state
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
        now = self.get_clock().now().nanoseconds * 1e-9
        stabilization_functions.imu_callback(
            msg,
            self.stabilization_state,
            now,
        )
        self.last_imu_orientation = self.stabilization_state['last_imu_orientation']

    def depth_callback(self, msg: Float32):
        now = self.get_clock().now().nanoseconds * 1e-9
        stabilization_functions.depth_callback(
            msg,
            self.stabilization_state,
            now,
            self.depth_timeout_s,
            self.velocity_filter_alpha,
            self.get_logger(),
        )

    def _depth_to_twist_control(self) -> float:
        return stabilization_functions._depth_to_twist_control(
            self.stabilization_state['current_depth'],
            self.stabilization_state['depth_rate'],
            self.stabilization_state['desired_depth'],
            self.depth_stabilization_params,
        )

    def control_callback(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        twist = Twist()

        imu_msg = self.stabilization_state['last_imu_msg']
        imu_sample_time = self.stabilization_state['imu_sample_time']
        if imu_msg is not None and imu_sample_time is not None:
            if now - imu_sample_time <= self.imu_timeout_s:
                angular_command = self._imu_to_twist_control(imu_msg)
                twist.angular.x = angular_command.angular.x
                twist.angular.y = angular_command.angular.y
                twist.angular.z = angular_command.angular.z
            else:
                self.get_logger().warn(
                    'IMU data is stale; orientation stabilization paused',
                    throttle_duration_sec=5.0,
                )

        current_depth = self.stabilization_state['current_depth']
        desired_depth = self.stabilization_state['desired_depth']
        depth_sample_time = self.stabilization_state['depth_sample_time']
        if (
            current_depth is not None
            and desired_depth is not None
            and depth_sample_time is not None
        ):
            if now - depth_sample_time <= self.depth_timeout_s:
                twist.linear.z = self._depth_to_twist_control()
            else:
                self.get_logger().warn(
                    'Depth data is stale; depth stabilization paused',
                    throttle_duration_sec=5.0,
                )

        self.publish_stabilization_commands(twist)

    def capture_callback(self, msg: Empty):
        stabilization_functions.capture_callback(
            msg,
            self.stabilization_state,
            self.imu_stabilization_params,
            self.depth_stabilization_params,
            self.get_logger(),
        )
        self.desired_orientation = self.stabilization_state['desired_orientation']

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
