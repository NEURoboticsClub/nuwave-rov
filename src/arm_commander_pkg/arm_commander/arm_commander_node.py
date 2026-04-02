import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs import msg
from std_msgs.msg import Float32, Float32MultiArray
import yaml
import os
import numpy as np

class ArmCommander(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('arm_commander')

        # Parameters
        self.declare_parameter('neutral_us', 1500.0)
        self.declare_parameter('min_us', 1100.0)
        self.declare_parameter('max_us', 1900.0)
        self.declare_parameter('max_force_n', 50.0)     # force at max PWM per arm motor
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter(
                'arm_config', 
                '/home/nuwave/nuwave-rov/src/arm_commander/config/arm_config.yaml'
                )
        self.declare_parameter('arm_topic', 'arm_commands')
        self.declare_parameter('n_arm_motors', 6)  
        
        self.neutral_us = float(self.get_parameter('neutral_us').value or 1500.0)
        self.min_us = float(self.get_parameter('min_us').value or 1100.0)
        self.max_us = float(self.get_parameter('max_us').value or 1900.0)
        self.max_force = float(self.get_parameter('max_force_n').value or 50.0)
        self.n_arm_motors = int(self.get_parameter('n_arm_motors').value or 6)
        rate = float(self.get_parameter('publish_rate_hz').value or 50.0)

        arm_config_path = self.get_parameter('arm_config').value
        arm_topic = self.get_parameter('arm_topic').value

        self.get_logger().info(f"Loading arm config from: {arm_config_path}")

        # Configure arm motors and Allocation
        # config = self.load_yaml(arm_config_path)

        # Subscribers / Publishers
        self.status_sub = self.create_subscription(Float32MultiArray, arm_topic, self.Status_Callback, 10)
        self.arm_motor_pubs = [] 
        for indx in range(self.n_arm_motors):
            pub = self.create_publisher(Float32, f'arm/arm_motor_{indx}', 10)
            self.arm_motor_pubs.append(pub)

        self.pwm_commands = np.full(self.n_arm_motors, self.neutral_us)

        self.create_timer(1.0 / rate, self.publish_arm_motors)
        self.get_logger().info("Arm Commander Initialized")

    def load_yaml(self, path):
        """Load a YAML file from a relative or absolute path."""
        if not os.path.exists(path):
            self.get_logger().warn(f"Config file not found: {path}")
            return {}
        with open(path, 'r') as f:
            return yaml.safe_load(f)    
        
    def Status_Callback(self, msg: Float32MultiArray):
        """
        Process a new velocity vector and turns them into PWM signals for the Arm Motors.
        """
        self.last_state_msg = msg
     
        arm_velocities = np.array(msg.data, dtype=float)

        self.pwm_commands = self.map_val_to_PWM(arm_velocities)

    
    # takes normalized game controller input (-1 to 1) and maps that to PWM range for each motor
    def map_val_to_PWM(self, arm_velocities) -> np.ndarray:
        
        pwm_commands = np.zeros(self.n_arm_motors)
        for i in range(self.n_arm_motors):
            if arm_velocities[i] > 0:
                pwm_commands[i] = self.neutral_us + arm_velocities[i] * (self.max_us - self.neutral_us)
            else:
                pwm_commands[i] = self.neutral_us + arm_velocities[i] * (self.neutral_us - self.min_us)

        return pwm_commands

    def publish_arm_motors(self):
        """
        Publish current PWM commands to each arm motor
        """
        for indx, pub in enumerate(self.arm_motor_pubs):
            msg = Float32()
            msg.data = float(self.pwm_commands[indx])
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
