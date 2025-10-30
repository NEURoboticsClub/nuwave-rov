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
from imu_driver import ImuDriver

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('ImuPublisher')

        # TODO: Parameters to interface IMU
        addr = self.declare_parameter('addr', 8,).value
        loop_freq = self.declare_parameter('control_loop_freq',4800).value
        
        # Setup
        try:
            self.imu = ImuDriver(addr,loop_freq)
        except:
            self.get_logger().error('Failed to initialize IMU')
        finally:
            self.get_logger().info(f"IMU initialized")
        
        self.publisher_ = self.create_publisher(Imu,'imu',10)

        timer_period = 1/loop_freq
        self.timer = self.create_timer(timer_period,self.timer_callback)

    def timer_callback(self):
        """
            Reads data from IMU and publishes it to the /imu topic
            Based on whatever control loop freq is provided
        """
        values = self.imu.read_data()
        acceleration = values['acceleration']
        velocity = values['velocity']
        magnetometer = values['magnetometer']
        quaternion = values['game_quaternion']
        imu_msg = Imu()
        # TODO: Write stuff to IMU message

        self.publisher_.publish(imu_msg)



def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)

    imu_publisher.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
