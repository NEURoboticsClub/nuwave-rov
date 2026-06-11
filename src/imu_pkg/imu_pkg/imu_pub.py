#!/usr/bin/env python3
# -*- coding: utf-8 -*-

########################################################################
# Run with ros2 run imu_pkg imu_pub addr:=<I2C address of IMU>
########################################################################

"""
Imports
"""
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from imu_pkg.imu_driver import ImuDriver
from geometry_msgs.msg import Quaternion
import numpy as np

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('ImuPublisher')

        # Declare parameter to ros so we can use it as argument 
        addr = self.declare_parameter('addr', 7).value # change this when change i2c pin (pin 3,4 -> 7, pin 27,28 -> 1)
        sampling_rate = self.declare_parameter('sampling_rate', 100).value

        self.q_mount = self.declare_parameter(
            'q_mount',
            [0.0, 0.0, -0.7071068, 0.7071068]
        ).value

        self._qm = np.array([self.q_mount[3], self.q_mount[0], self.q_mount[1], self.q_mount[2]])
        self._qm_inv = np.array([self._qm[0], -self._qm[1], -self._qm[2], -self._qm[3]])

        # Setup (with retry logic)
        while True:
            try:
                self.imu = ImuDriver(addr, sampling_rate)
                break
            except Exception as e:
                self.get_logger().error(f'IMU init failed: {e}')
                time.sleep(1.0)

        self.get_logger().info('IMU initialized')

        self.publisher_ = self.create_publisher(Imu, 'imu', 10)

        timer_period = 1 / sampling_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Helpers
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

    @staticmethod
    def _rotate_vec(q, v):
        """Rotate vector v by quaternion q (w, x, y, z)."""
        w, x, y, z = q
        vx, vy, vz = v

        return np.array([
            (1 - 2*y*y - 2*z*z)*vx + 2*(x*y - w*z)*vy + 2*(x*z + w*y)*vz,
            2*(x*y + w*z)*vx + (1 - 2*x*x - 2*z*z)*vy + 2*(y*z - w*x)*vz,
            2*(x*z - w*y)*vx + 2*(y*z + w*x)*vy + (1 - 2*x*x - 2*y*y)*vz,
        ])

    def timer_callback(self):
        """
        Reads data from IMU and publishes it to the /imu topic
        Based on whatever control loop freq is provided
        """

        if self.imu is None:
            return

        try:
            values = self.imu.read_data()
            linear_acceleration = values['linear_acceleration']
            # linear_velocity = values['linear_velocity']
            angular_velocity = values['angular_velocity']
            magnetometer = values['magnetometer']
            quaternion = values['game_quaternion']
        except Exception as e:
            self.get_logger().error(f'IMU read failed, skipping this cycle: {e}')
            return

        imu_msg = Imu()

        # Parse header with frame id and timestamp
        imu_msg.header.frame_id = 'imu_link'
        # Get current system time
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        q_chip = np.array([quaternion['real'], quaternion['i'], quaternion['j'], quaternion['k']])

        q_body = self._quat_mul(q_chip, self._qm_inv)
        imu_msg.orientation.w = float(q_body[0])
        imu_msg.orientation.x = float(q_body[1])
        imu_msg.orientation.y = float(q_body[2])
        imu_msg.orientation.z = float(q_body[3])

        gyro_body = self._rotate_vec(
            self._qm, [angular_velocity['gyro_x'],
                    angular_velocity['gyro_y'],
                    angular_velocity['gyro_z']]
        )
        imu_msg.angular_velocity.x = float(gyro_body[0])
        imu_msg.angular_velocity.y = float(gyro_body[1])
        imu_msg.angular_velocity.z = float(gyro_body[2])

        accel_body = self._rotate_vec(
            self._qm, [linear_acceleration['accel_x'],
                    linear_acceleration['accel_y'],
                    linear_acceleration['accel_z']]
        )
        imu_msg.linear_acceleration.x = float(accel_body[0])
        imu_msg.linear_acceleration.y = float(accel_body[1])
        imu_msg.linear_acceleration.z = float(accel_body[2])

        # imu_msg.linear_velocity.x = linear_velocity['vel_x']
        # imu_msg.linear_velocity.y = linear_velocity['vel_y']
        # imu_msg.linear_velocity.z = linear_velocity['vel_z']

        self.publisher_.publish(imu_msg)



def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
