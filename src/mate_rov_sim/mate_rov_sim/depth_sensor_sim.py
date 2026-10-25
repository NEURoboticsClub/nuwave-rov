#!/usr/bin/env python3
"""
depth_sensor_sim: the simulator's stand-in for rov_depth_sensor/depth_sensor_node
(MS5837-02BA), built from Gazebo's ground-truth odometry. Gazebo's built-in
pressure sensor models air, not water, so we compute hydrostatic pressure ourselves.

Subscribes:  odom_topic (default /sim/odom_ground_truth)  nav_msgs/Odometry
Publishes (same names, types and rate as depth_sensor_node):
  depth/depth        std_msgs/Float32            metres below the surface, POSITIVE DOWN
  depth/depth_array  std_msgs/Float32MultiArray  recent depths (same sliding window)
  depth/pressure     sensor_msgs/FluidPressure   absolute pressure in Pa
  depth/temperature  sensor_msgs/Temperature     deg C (constant)
"""
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float32, Float32MultiArray

# How the MS5837 driver turns pressure into depth (rov_depth_sensor/ms5837.py)
DRIVER_SURFACE_PA = 101300.0
DRIVER_DENSITY = 997.0
DRIVER_GRAVITY = 9.80665


class DepthSensorSim(Node):
    def __init__(self):
        super().__init__('depth_sensor_sim')
        self.declare_parameter('odom_topic', '/sim/odom_ground_truth')
        self.declare_parameter('publish_rate', 10.0)        # Hz, same as depth_sensor_node
        self.declare_parameter('depth_list_length', 10)
        self.declare_parameter('surface_z', 0.0)            # world z of the water surface [m]
        self.declare_parameter('sensor_offset_z', 0.0)      # sensor height above the CoM [m]
        self.declare_parameter('fluid_density', 997.0)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('atmospheric_pressure', 101325.0)
        self.declare_parameter('noise_stddev', 0.005)       # depth noise [m]
        self.declare_parameter('temperature', 20.0)         # deg C
        self.declare_parameter('frame_id', 'depth_sensor')

        gp = self.get_parameter
        self.depth_list_length = gp('depth_list_length').value
        self.surface = gp('surface_z').value
        self.offset = gp('sensor_offset_z').value
        self.rho_g = gp('fluid_density').value * gp('gravity').value
        self.p_atm = gp('atmospheric_pressure').value
        self.sigma = gp('noise_stddev').value
        self.temperature = gp('temperature').value
        self.frame_id = gp('frame_id').value

        self.z = None
        self.depth_list = []
        self.depth_pub = self.create_publisher(Float32, 'depth/depth', 10)
        self.depth_array_pub = self.create_publisher(Float32MultiArray, 'depth/depth_array', 10)
        self.pressure_pub = self.create_publisher(FluidPressure, 'depth/pressure', 10)
        self.temperature_pub = self.create_publisher(Temperature, 'depth/temperature', 10)
        self.create_subscription(Odometry, gp('odom_topic').value, self.on_odom, 10)
        self.create_timer(1.0 / gp('publish_rate').value, self.on_timer)

    def on_odom(self, msg):
        # (Ignores the small effect of tilt on the sensor offset.)
        self.z = msg.pose.pose.position.z + self.offset

    def on_timer(self):
        if self.z is None:
            return
        true_depth = self.surface - self.z
        pressure = self.p_atm + self.rho_g * max(true_depth, 0.0) + random.gauss(0.0, self.rho_g * self.sigma)
        depth = (pressure - DRIVER_SURFACE_PA) / (DRIVER_DENSITY * DRIVER_GRAVITY)

        # Same sliding window as depth_sensor_node (holds depth_list_length - 1 values)
        self.depth_list.append(depth)
        if len(self.depth_list) >= self.depth_list_length:
            self.depth_list.pop(0)

        self.depth_pub.publish(Float32(data=float(depth)))
        self.depth_array_pub.publish(Float32MultiArray(data=[float(d) for d in self.depth_list]))

        stamp = self.get_clock().now().to_msg()
        p = FluidPressure()
        p.header.stamp = stamp
        p.header.frame_id = self.frame_id
        p.fluid_pressure = float(pressure)
        self.pressure_pub.publish(p)

        t = Temperature()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.temperature = float(self.temperature)
        self.temperature_pub.publish(t)


def main():
    rclpy.init()
    node = DepthSensorSim()
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
