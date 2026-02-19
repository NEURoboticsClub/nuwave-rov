import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs import msg
from std_msgs.msg import Float32
import yaml
import os
import numpy as np

class ArmController(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('arm_controller')

        # Parameters
        self.declare_parameter('neutral_us', 1500.0)
        self.declare_parameter('min_us', 1100.0)
        self.declare_parameter('max_us', 1900.0)
        self.declare_parameter('max_force_n', 50.0)     # force at max PWM per arm motor
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter(
                'arm_config', 
                '/workspace/nuwave-rov/src/arm_controller/config/arm_config.yaml'
                )
        self.declare_parameter('arm_topic', '/arm')

        self.neutral_us = float(self.get_parameter('neutral_us').value or 1500.0)
        self.min_us = float(self.get_parameter('min_us').value or 1100.0)
        self.max_us = float(self.get_parameter('max_us').value or 1900.0)
        self.max_force = float(self.get_parameter('max_force_n').value or 50.0)
        rate = float(self.get_parameter('publish_rate_hz').value or 50.0)

        arm_config_path = self.get_parameter('arm_config').value
        arm_topic = self.get_parameter('arm_topic').value

        self.get_logger().info(f"Loading arm config from: {arm_config_path}")

        # Configure arm motors and Allocation
        config = self.load_yaml(arm_config_path)
        self.allocMatrix = self.compute_arm_allocation_matrix(config)
        self.n_arm_motors = self.allocMatrix.shape[1]
        self.allocMatrix_inverse = np.linalg.pinv(self.allocMatrix)

        # Subscribers / Publishers
        self.status_sub = self.create_subscription(Twist, arm_topic, self.Status_Callback, 10)
        self.arm_motor_pubs = [] 
        for indx in range(self.n_arm_motors):
            pub = self.create_publisher(Float32, f'arm/arm_motor_{indx}', 10)
            self.arm_motor_pubs.append(pub)

        # State
        self.lastest_twist = Twist()
        self.pwm_commands = np.full(self.n_arm_motors, self.neutral_us)
        # Listen to state
        # Scream to arm motors

        self.create_timer(1.0 / rate, self.publish_arm_motors)
        self.get_logger().info("Arm Controller Initialized")


    def load_yaml(self, config_path) -> dict:
        """Load a YAML file from a relative or absolute path."""
        if not os.path.exists(config_path):
            self.get_logger().warn(f"Config file not found: {config_path}")
            return {}
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def compute_arm_allocation_matrix(self, config : dict) -> np.ndarray:
        arm_motors = config['arm_motors']
        AllocMatrix = np.zeros((6, len(arm_motors)))

        for indx, arm_motor in enumerate(arm_motors):
            pos_m = np.array(arm_motor['position_m'], dtype=float)
            dir = np.array(arm_motor['direction'], dtype=float)

            dir = dir / np.linalg.norm(dir) # normalize the direction vector

            # Linear Force Contribution
            AllocMatrix[0:3, indx] = dir
            AllocMatrix[3:6, indx] = np.cross(pos_m, dir) 

            
        self.get_logger().info(f"Arm Motor Allocation Matrix:\n {AllocMatrix}")
        return AllocMatrix

    def Status_Callback(self, msg: Twist):
        """
        Process a new velocity vector and turns them into PWM signals for the Arm Motors.
        """
        self.last_state_msg = msg
        # Get twist msg
        # Twist -> msg -> force -> PWM
        # Convert to Numpy vectors, so easier to work with
        linearVel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        angularVel = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
        
        torqueList = self.map_twist_to_toque(linearVel, angularVel)
        # Publish PWM data
        self.pwm_commands = self.map_torque_to_PWM(torqueList)

    def map_twist_to_toque(self, linear : np.ndarray, angular : np.ndarray) -> np.ndarray:
        """
        Turns a twist msg -> a torque map for each arm motor
        """
        tau = np.concatenate([linear, angular])
        forces = self.allocMatrix_inverse @ tau # nx1 arm motor forces
        return forces
    
    def map_torque_to_PWM(self, forces : np.ndarray) -> np.ndarray:
        """
        Turns a list of torques -> to a list of PWM signals
        """

        normalized = np.clip(forces / self.max_force, -1, 1)
        
        pwm = np.where(
                normalized >= 0,
                self.neutral_us + normalized * (self.max_us - self.neutral_us),
                self.neutral_us + normalized * (self.neutral_us - self.min_us)
                )
        return pwm

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
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
