import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion, Twist
from nuwave_utils_pkg.file_helpers import load_yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from stabilization_pkg import stabilization_functions
from std_msgs.msg import Bool, Empty, Float32


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
        self.xy_stabilization_params = config.get('xy_stabilization', {}) or {}

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
        self.manual_vertical_active = False
        self.stabilization_state = {
            'desired_orientation': self.desired_orientation,
            'desired_depth': None,
            'depth_capture_pending': False,
            'last_imu_orientation': None,
            'last_imu_msg': None,
            'imu_sample_time': None,
            'current_depth': None,
            'depth_rate': 0.0,
            'depth_sample_time': None,
            # Body-frame XY estimate, setpoint, and previous IMU reference.
            'current_position': None,
            'current_velocity': None,
            'desired_position': None,
            'xy_orientation': None,
            'xy_sample_time': None,
        }

        # Subscribers / Publishers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.depth_sub = self.create_subscription(
            Float32, '/depth/depth', self.depth_callback, 10
        )
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.manual_vertical_sub = self.create_subscription(
            Bool, '/controls/manual_vertical_active', self.manual_vertical_callback, qos_state
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
        # Integrate once per IMU sample, not once per control timer tick.
        stabilization_functions.update_xy_estimate(
            msg, self.stabilization_state, self.imu_timeout_s
        )

    def _xy_to_twist_control(self) -> Twist:
        state = self.stabilization_state
        return stabilization_functions._xy_to_twist_control(
            state['current_position'],
            state['current_velocity'],
            state['desired_position'],
            self.xy_stabilization_params,
        )

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

    def manual_vertical_callback(self, msg: Bool):
        if msg.data:
            self.stabilization_state['depth_capture_pending'] = False
        elif self.manual_vertical_active:
            self.stabilization_state['depth_capture_pending'] = True
        self.manual_vertical_active = msg.data

    def _depth_to_twist_control(self) -> float:
        return stabilization_functions.depth_control(
            self.stabilization_state,
            self.manual_vertical_active,
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
                # Add XY corrections only when an estimate and target exist.
                if (
                    self.stabilization_state['current_position'] is not None
                    and self.stabilization_state['desired_position'] is not None
                ):
                    xy_command = self._xy_to_twist_control()
                    twist.linear.x = xy_command.linear.x
                    twist.linear.y = xy_command.linear.y
            else:
                # Require a new XY capture after an IMU outage.
                self.stabilization_state['current_position'] = None
                self.stabilization_state['current_velocity'] = None
                self.stabilization_state['desired_position'] = None
                self.stabilization_state['xy_sample_time'] = None
                self.get_logger().warn(
                    'IMU data is stale; orientation and XY stabilization paused; '
                    'XY recapture required',
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
            xy_stabilization_params=self.xy_stabilization_params,
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
