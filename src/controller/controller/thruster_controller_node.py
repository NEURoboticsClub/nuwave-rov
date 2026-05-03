import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs import msg
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import numpy as np

class ThrusterController(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('thruster_controller')

        # Parameters

        # Four thrusters control linear velocity, and they are pointed at 45 degrees. To get an accurate force to pwm use the equation below
        thruster_max = 4 / np.sqrt(2) 
        self.declare_parameter('max_x_n', thruster_max)
        self.declare_parameter('max_y_n', thruster_max)
        self.declare_parameter('max_z_n', 4.0)
        self.declare_parameter('max_roll_nm', 1.0)
        self.declare_parameter('max_pitch_nm', 1.0)
        self.declare_parameter('max_yaw_nm', 1.0)

        self.wrench_scale = np.array([
            float(self.get_parameter('max_x_n').value),
            float(self.get_parameter('max_y_n').value),
            float(self.get_parameter('max_z_n').value),
            float(self.get_parameter('max_roll_nm').value),
            float(self.get_parameter('max_pitch_nm').value),
            float(self.get_parameter('max_yaw_nm').value),
        ])

        self.declare_parameter('neutral_us', 1500.0)
        self.declare_parameter('min_us', 1100.0)
        self.declare_parameter('max_us', 1900.0)
        self.declare_parameter('max_force_n', 1.0)     # force at max PWM per thruster
        self.declare_parameter('publish_rate_hz', 50.0)
        pkg_share = get_package_share_directory('controller')
        self.declare_parameter(
                'thruster_config', 
                os.path.join(pkg_share, 'config', 'thruster_config.yaml')
                )
        self.declare_parameter('thruster_topic', '/thruster')

        self.neutral_us = float(self.get_parameter('neutral_us').value or 1500.0)
        self.min_us = float(self.get_parameter('min_us').value or 1100.0)
        self.max_us = float(self.get_parameter('max_us').value or 1900.0)
        self.max_force = float(self.get_parameter('max_force_n').value or 50.0)
        rate = float(self.get_parameter('publish_rate_hz').value or 50.0)

        thruster_config_path = self.get_parameter('thruster_config').value
        thruster_topic = self.get_parameter('thruster_topic').value

        self.get_logger().info(f"Loading thruster config from: {thruster_config_path}")

        # Configure Thrusters and Allocation
        config = self.load_yaml(thruster_config_path)
        self.allocMatrix = self.compute_thruster_allocation_matrix(config)
        self.n_thrusters = self.allocMatrix.shape[1]
        self.allocMatrix_inverse = np.linalg.pinv(self.allocMatrix)

        # Subscribers / Publishers
        self.status_sub = self.create_subscription(Twist, "velocity_commands", self.Status_Callback, 10)
        self.thruster_pubs = [] 
        for indx in range(self.n_thrusters):
            pub = self.create_publisher(Float32, f'thruster/thruster_{indx}', 10)
            self.thruster_pubs.append(pub)

        # State
        self.lastest_twist = Twist()
        self.pwm_commands = np.full(self.n_thrusters, self.neutral_us)
        # Listen to state
        # Scream to thrusters

        self.create_timer(1.0 / rate, self.publish_thrusters)
        self.get_logger().info("Thruster Controller Initialized")


    def load_yaml(self, config_path) -> dict:
        """Load a YAML file from a relative or absolute path."""
        if not os.path.exists(config_path):
            self.get_logger().warn(f"Config file not found: {config_path}")
            return {}
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def compute_thruster_allocation_matrix(self, config : dict) -> np.ndarray:

        thrusters = config['thrusters']
        AllocMatrix = np.zeros((6, len(thrusters)))

        for indx, thruster in enumerate(thrusters):
            pos_m = np.array(thruster['position_m'], dtype=float)
            dir = np.array(thruster['direction'], dtype=float)

            dir = dir / np.linalg.norm(dir) # normalize the direction vector

            # Linear Force Contribution
            AllocMatrix[0:3, indx] = dir
            AllocMatrix[3:6, indx] = np.cross(pos_m, dir) 

            
        self.get_logger().info(f"Thruster Allocation Matrix:\n {AllocMatrix}")
        return AllocMatrix

    def Status_Callback(self, msg: Twist):
        """
        Process a new velocity vector and turns them into PWM signals for the Thrusters.
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
        Turns a twist msg -> a torque map for each thruster
        """
        tau_norm = np.concatenate([linear, angular])
        tau = tau_norm * self.wrench_scale
        forces = self.allocMatrix_inverse @ tau
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

    def publish_thrusters(self):
        """
        Publish current PWM commands to each thruster
        """
        for indx, pub in enumerate(self.thruster_pubs):
            msg = Float32()
            msg.data = float(self.pwm_commands[indx])
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
