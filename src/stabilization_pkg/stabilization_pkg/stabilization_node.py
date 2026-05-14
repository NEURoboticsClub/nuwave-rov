import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_share_directory
from nuwave_utils_pkg.file_helpers import load_yaml
import os
import numpy as np

class StabilizationNode(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('stabilization_node')

        # Parameters
        self.declare_parameter('publish_rate_hz', 50.0)

        pkg_share = get_package_share_directory('stabilization_pkg')
        self.declare_parameter(
                'stabilization_config', 
                os.path.join(pkg_share, 'config', 'stabilization_config.yaml')
                )
        self.declare_parameter('stabilization_topic', 'stabilization_commands')

        rate = float(self.get_parameter('publish_rate_hz').value)

        stabilization_config_path = self.get_parameter('stabilization_config').value
        stabilization_topic = self.get_parameter('stabilization_topic').value

        self.get_logger().info(f"Loading stabilization config from: {stabilization_config_path}")

        config = load_yaml(stabilization_config_path)

        self.imu_stabilization_params = config.get('imu_stabilization', {})
        self.depth_stabilization_params = config.get('depth_stabilization', {})

        # Subscribers / Publishers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.sta_pub = self.create_publisher(Twist, stabilization_topic, 10)

        self.desired_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.get_logger().info("Stabilization Node Initialized")
        
    @staticmethod
    def _quat_mul(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])

    def set_desired_orientation(self, q: Quaternion) -> bool:
        """
        Update the attitude setpoint; Input is normalized
        """
        n = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) ** 0.5
        if n < 1e-6:
            self.get_logger().warn(
                "Rejected desired orientation: quaternion has near-zero norm"
            )
            return False

        self.desired_orientation = Quaternion(
            w=q.w / n,
            x=q.x / n,
            y=q.y / n,
            z=q.z / n,
        )
        self.get_logger().info(
            f"Desired orientation set to "
            f"(w={self.desired_orientation.w:.3f}, "
            f"x={self.desired_orientation.x:.3f}, "
            f"y={self.desired_orientation.y:.3f}, "
            f"z={self.desired_orientation.z:.3f})"
        )
        return True

    def _imu_to_twist_control(self, imu_msg: Imu) -> Twist:
        """
        Compute a body-frame angular-velocity command that drives current
        orientation toward self.desired_orientation
        """
        cur = imu_msg.orientation
        des = self.desired_orientation
        gyro = imu_msg.angular_velocity

        q_cur = np.array([cur.w, cur.x, cur.y, cur.z])
        q_des = np.array([des.w, des.x, des.y, des.z])

        n = np.linalg.norm(q_cur)
        if n < 1e-6:
            return Twist()
        q_cur /= n
        q_cur_inv = np.array([q_cur[0], -q_cur[1], -q_cur[2], -q_cur[3]])

        q_err_body = self._quat_mul(q_cur_inv, q_des)
        if q_err_body[0] < 0.0:
            q_err_body = -q_err_body
        err_vec = 2.0 * q_err_body[1:]

        p = float(self.imu_stabilization_params.get('p', 1.0))
        d = float(self.imu_stabilization_params.get('d', 0.0))

        twist = Twist()
        twist.angular.x = float(p * err_vec[0] - d * gyro.x)
        twist.angular.y = float(p * err_vec[1] - d * gyro.y)
        twist.angular.z = float(p * err_vec[2] - d * gyro.z)
        return twist

    def imu_callback(self, msg: Imu):
        """
        Callback for IMU data.
        """
        self.publish_stabilization_commands(self._imu_to_twist_control(msg))

    def publish_stabilization_commands(self, msg: Twist):
        """
        Publish new Twist commands to houston
        """
        self.sta_pub.publish(msg)


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
