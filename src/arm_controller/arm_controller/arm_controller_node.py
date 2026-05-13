import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs import msg
from std_msgs.msg import Float32, Float32MultiArray
from ament_index_python.packages import get_package_share_directory
from nuwave_utils_pkg.file_helpers import load_yaml
import os
import numpy as np

class ArmController(Node):
    def __init__(self):
        # ROS2 stuff
        super().__init__('arm_controller')

        # Parameters
        self.declare_parameter('publish_rate_hz', 50.0)

        pkg_share = get_package_share_directory('arm_controller')
        self.declare_parameter(
                'arm_config', 
                os.path.join(pkg_share, 'config', 'arm_config.yaml')
                )
        self.declare_parameter('arm_topic', 'arm_commands')

        rate = float(self.get_parameter('publish_rate_hz').value)

        arm_config_path = self.get_parameter('arm_config').value
        arm_topic = self.get_parameter('arm_topic').value

        self.get_logger().info(f"Loading arm config from: {arm_config_path}")

        config = load_yaml(arm_config_path)
        self.pwm_range_table = self._configure_arm_motor_pwm_range(config)
        self.n_arm_motors = self.pwm_range_table.shape[0]

        # Subscribers / Publishers
        self.status_sub = self.create_subscription(Float32MultiArray, arm_topic, self.Status_Callback, 10)
        self.arm_motor_pubs = [] 
        for indx in range(self.n_arm_motors):
            pub = self.create_publisher(Float32, f'arm/arm_motor_{indx}', 10)
            self.arm_motor_pubs.append(pub)
        
        # Init command send neutral to all arm motors
        self.pwm_commands = self.pwm_range_table[:,1]

        self.create_timer(1.0 / rate, self.publish_arm_motors)
        self.get_logger().info("Arm Controller Initialized")
        
    def Status_Callback(self, msg: Float32MultiArray):
        """
        Process a new velocity vector and turns them into PWM signals for the Arm Motors.
        """
        self.last_state_msg = msg
     
        arm_velocities = np.array(msg.data, dtype=float)

        self.pwm_commands = self.map_val_to_PWM(arm_velocities)

    def _configure_arm_motor_pwm_range(self, config : dict) -> np.ndarray:
        """
        Each arm motor has a neutral_us, min_us, max_us configured.
        When we get the force each arm motor should be outputting we want to map it along the bounds of that, specific arm motor.
        Returns: A list of bounds for the arm motor at index i
        """
        
        arm_motors = config.get('arm_motors', [])
        if not arm_motors:
            raise ValueError(f"No arm_motors found in config")
        table = np.zeros((len(arm_motors), 3))
        for indx, arm_motor in enumerate(arm_motors):
            try:
                table[indx, 0] = float(arm_motor['min_us'])
                table[indx, 1] = float(arm_motor['neutral_us'])
                table[indx, 2] = float(arm_motor['max_us'])
            except KeyError as e:
                raise ValueError(
                        f"Arm motor {indx} is missing required PWM field {e}. "
                        f"Each arm motor needs min_us, neutral_us, and max_us. "
                        )
        
        return table

    
    # takes normalized game controller input (-1 to 1) and maps that to PWM range for each motor
    def map_val_to_PWM(self, arm_velocities) -> np.ndarray:
        # These are nx1 vectors for each thrusters configured pwm ranges
        min_us = self.pwm_range_table[:, 0] 
        neutral_us = self.pwm_range_table[:, 1]
        max_us = self.pwm_range_table[:, 2]
        
        # This normalizes the forces to be within -1 and 1
        normalized = np.clip(arm_velocities, -1, 1)
        
        # If normalized >= 0, which is forward we linear interp the pwm to be some point between neutral and max
        # Same is applied for reverse but with neutral_us being the leading term.
        pwm = np.where(
            normalized >= 0,
            neutral_us + normalized * (max_us - neutral_us),
            neutral_us + normalized * (neutral_us - min_us)
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
