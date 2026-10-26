#!/usr/bin/env python3
"""
imu_sim: the simulator's stand-in for imu_pkg/imu_pub (BNO08x).

Gazebo's IMU reports specific force (gravity included: +9.81 on z when sitting
still), while imu_pub publishes the BNO08x "linear acceleration" report, which
has gravity removed. This node converts one into the other so stabilization_pkg
and the web GUI see the same thing they do on the real ROV.

The Gazebo IMU sits at the CoM aligned with base_link, which is already the
NUWave body frame (x right, y forward, z up), so imu_pub's q_mount correction
is not needed here.

Subscribes:  input_topic (default /sim/imu_raw)  sensor_msgs/Imu  from Gazebo
Publishes:   output_topic (default /imu)         sensor_msgs/Imu  frame imu_link
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


def rotate_inverse(q, v):
    """Rotate vector v by the inverse of quaternion q = (w, x, y, z): world -> body."""
    w, x, y, z = q
    r = np.array([
        [1 - 2*y*y - 2*z*z, 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*x*x - 2*z*z, 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*x*x - 2*y*y],
    ])
    return r.T @ np.asarray(v, dtype=float)


class ImuSim(Node):
    def __init__(self):
        super().__init__('imu_sim')
        self.declare_parameter('input_topic', '/sim/imu_raw')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('remove_gravity', True)
        self.declare_parameter('gravity', 9.81)

        gp = self.get_parameter
        self.frame_id = gp('frame_id').value
        self.remove_gravity = gp('remove_gravity').value
        self.g = gp('gravity').value

        self.pub = self.create_publisher(Imu, gp('output_topic').value, 10)
        self.create_subscription(Imu, gp('input_topic').value, self.on_imu, 10)

    def on_imu(self, msg):
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()  # wall time, like imu_pkg
        out.header.frame_id = self.frame_id
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity

        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        if self.remove_gravity:
            o = msg.orientation
            a = a - rotate_inverse((o.w, o.x, o.y, o.z), (0.0, 0.0, self.g))
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = (float(v) for v in a)
        # Covariances stay zero, like imu_pub.
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
