import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import numpy as np

class Houston(Node):
    """Status node to map joytick inputs to Twist commands."""

    def __init__(self):
        super().__init__('houston')

        pkg_share = get_package_share_directory('houston_pkg')
        
        # === Parameters ===
        self.declare_parameter('joy_config', os.path.join(pkg_share, 'config', 'joystick_config.yaml'))
        self.declare_parameter('joy_thruster', '/joy_thruster')
        self.declare_parameter('joy_arm', '/joy_arm')

        joy_config_path = self.get_parameter('joy_config').value
        joy_thruster = self.get_parameter('joy_thruster').value
        joy_arm = self.get_parameter('joy_arm').value

        self.get_logger().info(f"Loading joystick config from: {joy_config_path}")
        # === Load configurations ===
        self.joy_map = self.load_yaml(joy_config_path)
        
        # === Subscribers / Publishers ===
        self.thruster_joy_sub = self.create_subscription(Joy, joy_thruster, self.joy_thruster_callback, 10)
        self.arm_joy_sub = self.create_subscription(Joy, joy_arm, self.joy_arm_callback, 10)
        self.twist_pub = self.create_publisher(Twist, "velocity_commands", 10)
        self.arm_pub = self.create_publisher(Float32MultiArray, "arm_commands", 10)
        
        # === Internal state ===
        self.last_joy_thruster_msg = None
        self.last_joy_arm_msg = None

        self.get_logger().info("Houston Initialized")

    def load_yaml(self, path):
        """Load a YAML file from a relative or absolute path."""
        if not os.path.exists(path):
            self.get_logger().warn(f"Config file not found: {path}")
            return {}
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def joy_arm_callback(self, msg: Joy):
        """Handle arm joystick input"""
        self.last_joy_arm_msg = msg
        return_map = self.parse_joystick(msg, cfg_type="arm_control")
        self.get_logger().info(str(return_map))

        self.publish_arm_commands(return_map["axis"], return_map["button"])

    def joy_thruster_callback(self, msg: Joy):
        """Handle thruster joystick input"""
        self.last_joy_thruster_msg = msg
        return_map = self.parse_joystick(msg, cfg_type="thruster_control")
        self.get_logger().info(str(return_map))

        # Publish the result
        self.publish_twist(return_map["axis"], return_map["button"])

    def parse_joystick(self, msg: Joy, cfg_type: str) -> dict:
        cfg_list = self.joy_map.get(cfg_type, [])

        if not isinstance(cfg_list, list):
            return {} # config malformed

        axis_values = {}
        button_values = {}
        for cfg in cfg_list:
            # --- Axis entry ---
            if "axis" in cfg:
                axis_name = cfg["axis"]
                axis_index = int(cfg.get("input", 0))
                invert = bool(cfg.get("invert", False))
                sensitivity = float(cfg.get("sensitivity", 1.0))
                scale = cfg.get("scale", "linear")

                raw = msg.axes[axis_index] if axis_index < len(msg.axes) else 0.0
                if invert:
                    raw *= -1.0

                val = raw * sensitivity

                # Optional: support your "scale" field (keep simple)
                if scale == "logarithmic":
                    # compress near 0, preserve sign
                    val = np.sign(val) * np.log1p(abs(val))

                axis_values[axis_name] = float(val)
                continue

            # --- Button entry ---
            if "button" in cfg:
                btn_name = cfg["button"]
                btn_index = int(cfg.get("input", 0))
                invert = bool(cfg.get("invert", False))

                raw = msg.buttons[btn_index] if btn_index < len(msg.buttons) else 0
                pressed = (1 - raw) if invert else raw
                button_values[btn_name] = int(pressed)
                continue

        return {"axis": axis_values, "button": button_values}

    def publish_twist(self, axis_values, button_values):
        """Publish the twist velocity command."""
        msg = Twist()
        msg.angular.x = float(axis_values['pitch'])
        msg.angular.y = float(button_values['roll_right']) - float(button_values['roll_left'])
        msg.angular.z = float(axis_values['yaw'])


        msg.linear.x = float(axis_values['strafe'])
        msg.linear.y = float(axis_values['drive_forward'])
        msg.linear.z = (float(axis_values['up']) - float(axis_values['down'])) * 0.5

        self.twist_pub.publish(msg)


    def publish_arm_commands(self, axis_values, button_values):
        """Publish arm control commands."""
        # 6 element array: [axis1 - axis6]
        # Create and publish arm commands as a 6-element array
        
        msg = Float32MultiArray()
        msg.data = [
            float(axis_values.get('base_yaw_input', 0.0)),
            float(axis_values.get('base_pitch_input', 0.0)),
            float(axis_values.get('elbow_pitch_input', 0.0)),
            float(axis_values.get('wrist_yaw_input', 0.0)),
            (float(button_values.get('wrist_up_input', 0.0)) - float(button_values.get('wrist_down_input', 0.0))),
            (float(axis_values.get('claw_open_input', 0.0)) - float(axis_values.get('claw_close_input', 0.0))) * 0.5, 
        ]
        self.arm_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Houston()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
