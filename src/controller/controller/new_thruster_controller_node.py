import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import yaml
import os
import numpy as np

# Helper Functions
def compute_thruster_allocation_matrix(config : str) -> np.ndarray:
    A = np.array(np.zeros(3))
    return A



class ThrusterController(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('thruster_controller')

        # Parameters
        self.declare_parameter('thruster_config', 'path/to/file')
        self.declare_parameter('thruster_topic', '/thruster')

        thruster_config_path = self.get_parameter('thruster_config').value
        thruster_topic = self.get_parameter('thruster_topic').value

        self.get_logger().info(f"Loading thruster config from: {thruster_config_path}")
        # Configure Thrusters and Allocation

        # Subscribers / Publishers
        # Initialize Allocation Matrix
        # Listen to state
        # Scream to thrusters
        """TBA"""
    def Status_Callback(self, msg: Twist):
        """
        Process a new velocity vector and turns them into PWM signals for the Thrusters.
        """
        # Get twist msg
        # Twist -> msg -> force -> PWM
        # Convert to Numpy vectors, so easier to work with
        linearVel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        angularVel = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
        
        torqueList = self.map_twist_to_toque(linearVel, angularVel)
        PWMList = self.map_torque_to_PWM(torqueList)
        # Publish PWM data

    def map_twist_to_toque(self, linear : np.ndarray, angular : np.ndarray):
        """
        Turns a twist msg -> a torque map for each thruster
        """
    def map_torque_to_PWM(self, torqueList):
        """
        Turns a list of torques -> to a list of PWM signals
        """

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
