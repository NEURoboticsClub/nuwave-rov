#!/usr/bin/env python3
# -*- coding: utf-8 -*-

########################################################################
# Run with ros2 run imu_pkg imu_pub addr:=<I2C address of IMU>
########################################################################

"""
Imports
"""
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu
from imu_pkg.imu_driver import ImuDriver
from rclpy.clock import Clock

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('ImuPublisher')

        # Declare parameter to ros so we can use it as argument 
        addr = self.declare_parameter('addr', 7).value # change this when change i2c pin (pin 3,4 -> 7, pin 27,28 -> 1)
        sampling_rate = self.declare_parameter('sampling_rate',4800).value # why sampling rate 4800 lol, this is not baudrate

        # Setup
        try:
            self.imu = ImuDriver(addr,sampling_rate)
            self.get_logger().info(f"IMU initialized")
        except Exception as e:
            self.imu = None
            self.get_logger().error(f'Failed to initialize IMU: {e}')

        self.publisher_ = self.create_publisher(Imu,'imu',10)

        timer_period = 1/sampling_rate
        self.timer = self.create_timer(timer_period,self.timer_callback)

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
            linear_velocity = values['linear_velocity']
            angular_velocity = values['angular_velocity']
            magnetometer = values['magnetometer']
            quaternion = values['game_quaternion']
        except Exception as e:
            self.get_logger().error(f'IMU read failed, skipping this cycle: {e}')
            return

        imu_msg = Imu()

        # Parse header with frame id and timestamp
        imu_msg.header.frame_id = 'IMU_Frame'
        # Get current system time
        clock = Clock()
        now = clock.now()
        imu_msg.header.stamp.sec = now.seconds_nanoseconds()[0]
        imu_msg.header.stamp.nanosec = now.seconds_nanoseconds()[1]

        # Parse imu data
        imu_msg.angular_velocity.x = angular_velocity['gyro_x']
        imu_msg.angular_velocity.y = angular_velocity['gyro_y']
        imu_msg.angular_velocity.z = angular_velocity['gyro_z']

        imu_msg.linear_acceleration.x = linear_acceleration['accel_x']
        imu_msg.linear_acceleration.y = linear_acceleration['accel_y']
        imu_msg.linear_acceleration.z = linear_acceleration['accel_z']

        imu_msg.orientation.x = quaternion['i']
        imu_msg.orientation.y = quaternion['j']
        imu_msg.orientation.z = quaternion['k']
        imu_msg.orientation.w = quaternion['real']

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
