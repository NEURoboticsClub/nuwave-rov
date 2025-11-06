#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float32, Float32MultiArray
import time
from . import mock_ms5837 as ms5837  # Force mock


class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor_node')
        
        # Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz (your CONTROL_LOOP_FREQ)
        self.declare_parameter('depth_list_length', 10)  # Averaging window size
        self.declare_parameter('i2c_bus', 1)  # I2C bus number
        
        rate = self.get_parameter('publish_rate').value
        self.depth_list_length = self.get_parameter('depth_list_length').value
        i2c_bus = self.get_parameter('i2c_bus').value
        
        # Initialize MS5837 sensor
        self.sensor = ms5837.MS5837_02BA(i2c_bus)
        
        # Keep trying to initialize sensor (like your while loop)
        while not self.sensor.init():
            self.get_logger().error("Error: depth sensor could not be initialized")
            time.sleep(1.0)
        
        self.get_logger().info("MS5837 depth sensor initialized successfully")
        
        # Depth averaging list (your depth_list)
        self.depth_list = []
        
        # Publishers
        self.depth_pub = self.create_publisher(
            Float32,
            'depth/depth',  # Current depth
            10)
        
        self.depth_array_pub = self.create_publisher(
            Float32MultiArray,
            'depth/depth_array',  # Array of recent depths (your depth_list)
            10)
        
        self.pressure_pub = self.create_publisher(
            FluidPressure,
            'depth/pressure',
            10)
        
        self.temperature_pub = self.create_publisher(
            Temperature,
            'depth/temperature',
            10)
        
        # Timer to read sensor at specified rate (your control loop)
        self.timer = self.create_timer(1.0 / rate, self.read_and_publish)
        
        self.get_logger().info(f'Depth sensor node started at {rate} Hz')
    
    def read_depth(self):
        """
        Your original read_depth() function, adapted
        """
        if self.sensor.read():
            # Log detailed info (like your print statement)
            self.get_logger().debug(
                f"P: {self.sensor.pressure():.1f} mbar | "
                f"Depth: {self.sensor.depth():.3f} m | "
                f"T: {self.sensor.temperature():.2f} C"
            )
            return self.sensor.depth()
        else:
            self.get_logger().error("Sensor read failed!")
            raise LookupError("Encountered error reading from depth sensor")
    
    def read_and_publish(self):
        """
        Main control loop - replaces your while True loop
        """
        try:
            # Read depth (your read_depth call)
            depth = self.read_depth()
            
            # Update depth list (your sliding window logic)
            self.depth_list.append(depth)
            if len(self.depth_list) >= self.depth_list_length:
                self.depth_list.pop(0)
            
            # Publish current depth
            depth_msg = Float32()
            depth_msg.data = float(depth)
            self.depth_pub.publish(depth_msg)
            
            # Publish depth array (equivalent to your JSON message)
            depth_array_msg = Float32MultiArray()
            depth_array_msg.data = self.depth_list
            self.depth_array_pub.publish(depth_array_msg)
            
            # Publish pressure
            pressure_msg = FluidPressure()
            pressure_msg.header.stamp = self.get_clock().now().to_msg()
            pressure_msg.header.frame_id = 'depth_sensor'
            pressure_msg.fluid_pressure = self.sensor.pressure() * 100.0  # mbar to Pa
            pressure_msg.variance = 0.0
            self.pressure_pub.publish(pressure_msg)
            
            # Publish temperature
            temp_msg = Temperature()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = 'depth_sensor'
            temp_msg.temperature = self.sensor.temperature()
            temp_msg.variance = 0.0
            self.temperature_pub.publish(temp_msg)
            
            self.get_logger().debug(f"Published depth: {depth:.3f} m")
            
        except Exception as err:
            self.get_logger().error(f"Error in depth sensor: {str(err)}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
