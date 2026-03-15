import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from power_monitor_pkg.power_monitor_driver import INA226

class PowerMonitorPublisher(Node):
    def __init__(self):
        super().__init__('power_monitor_publisher')

        #Deafualts
        self.declare_parameter('i2c_address', 0x4F)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('shunt_resistor', 0.004)
        self.declare_parameter('max_expected_current', 17.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('topic_prefix', 'power_monitor')

        # Read parameters
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        shunt_resistor = self.get_parameter('shunt_resistor').get_parameter_value().double_value
        max_current = self.get_parameter('max_expected_current').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        topic_prefix = self.get_parameter('topic_prefix').get_parameter_value().string_value

        # Create publishers
        self.pub_voltage = self.create_publisher(Float32, f'{topic_prefix}/bus_voltage', 10)
        self.pub_current = self.create_publisher(Float32, f'{topic_prefix}/current', 10)
        self.pub_power = self.create_publisher(Float32, f'{topic_prefix}/power', 10)
        self.pub_shunt = self.create_publisher(Float32, f'{topic_prefix}/shunt_voltage', 10)
        
        # Initialize the INA226
        try:
            self.ina226 = INA226(i2c_address=i2c_address, bus_number=i2c_bus)  # Adjust address if needed
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
