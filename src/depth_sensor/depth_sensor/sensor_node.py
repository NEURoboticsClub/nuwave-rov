import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from depth_sensor import ms5837

class DepthSensor(Node):
    """
    Publishes data from depth sensor
    This class reads data from the sensor using the ms5837.py driver file 
        and then publishes required data on /barometer/.. topics
    
    Naming convention for subtopics: barometer/<reading_type>/<reading_subtype_(optional)
    Example: barometer/pressure/psi | barometer/temperature/

    """
    def __init__(self):
        super().__init__('depth_sensor')
        # add more publishers to publish more stuff
        self.pressure_publisher_ = self.create_publisher(Float32, 'barometer/pressure', 10)
        self.temperature_publisher_ = self.create_publisher(Float32, 'barometer/temperature', 10)
        self.depth_publisher_ = self.create_publisher(Float32, 'barometer/depth', 10)
        
        # timing for publishing (this publishes once evert 0.1 seconds)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # initializing sensor
        self.sensor = ms5837.MS5837_02BA()

    def timer_callback(self):
        """
            Called at every time_period
            Reads data from sensor and publishes it to respective topics
        """
        # change stuff here if other things need to be published
        
        # read
        if self.sensor.read():
            pressure = Float32(self.sensor.pressure())
            depth = Float32(self.sensor.depth())
            temperature = Float32(self.sensor.temperature())
        
        # publish
        self.pressure_publisher_.publish(pressure)
        self.temperature_publisher_.publish(temperature)
        self.depth_publisher_.publish(depth)
        

def main(args=None):
    rclpy.init(args=args)

    depth_publisher = DepthSensor()

    rclpy.spin(depth_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
