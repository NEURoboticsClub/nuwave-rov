import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from power_monitor_driver import INA226

class PowerMonitorPublisher(Node):
    def __init__(self):
        super().__init__('power_monitor_publisher')
        
        # Create publishers for each measurement
        self.pub_voltage = self.create_publisher(Float32, 'power_monitor/bus_voltage', 10)
        self.pub_current = self.create_publisher(Float32, 'power_monitor/current', 10)
        self.pub_power = self.create_publisher(Float32, 'power_monitor/power', 10)
        self.pub_shunt = self.create_publisher(Float32, 'power_monitor/shunt_voltage', 10)
        
        # Initialize the INA226
        try:
            self.ina226 = INA226(i2c_address=0x40, bus_number=1)  # Adjust address if needed
            self.ina226.configure()
            self.ina226.calibrate(max_expected_current=17.0)  # Adjust for your system
            self.get_logger().info('INA226 initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize INA226: {e}')
            raise
        
        # Create timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        """Read and publish power monitor data"""
        try:
            data = self.ina226.read_all()
            
            # Publish each measurement
            msg_voltage = Float32()
            msg_voltage.data = data['bus_voltage']
            self.pub_voltage.publish(msg_voltage)
            
            msg_current = Float32()
            msg_current.data = data['current']
            self.pub_current.publish(msg_current)
            
            msg_power = Float32()
            msg_power.data = data['power']
            self.pub_power.publish(msg_power)
            
            msg_shunt = Float32()
            msg_shunt.data = data['shunt_voltage']
            self.pub_shunt.publish(msg_shunt)
            
            self.get_logger().info(
                f'V: {data["bus_voltage"]:.3f}V, '
                f'I: {data["current"]:.3f}A, '
                f'P: {data["power"]:.3f}W'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error reading INA226: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()