import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import yaml
import os
import numpy as np

class Houston(Node):
    """Status node to map joytick inputs to Twist commands."""

    def __init__(self):
        super().__init__('houston')

        # === Parameters ===
        self.declare_parameter('joy_config', '/home/nuwave/nuwave-rov/install/houston_pkg/share/houston_pkg/config/joystick_config.yaml')
        self.declare_parameter('joy_topic', '/joy')

        joy_config_path = self.get_parameter('joy_config').value
        joy_topic = self.get_parameter('joy_topic').value

        self.get_logger().info(f"Loading joystick config from: {joy_config_path}")
        # === Load configurations ===
        self.joy_map = self.load_yaml(joy_config_path)
        
        # === Subscribers / Publishers ===
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.twist_pub = self.create_publisher(Twist, "velocity_commands", 10)
        
        # === Internal state ===
        self.last_joy_msg = None

        self.get_logger().info("Houston Initialized")

    def load_yaml(self, path):
        """Load a YAML file from a relative or absolute path."""
        if not os.path.exists(path):
            self.get_logger().warn(f"Config file not found: {path}")
            return {}
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def joy_callback(self, msg: Joy):
        """Handle joystick input."""
        self.last_joy_msg = msg
        axis_values, button_values = self.parse_joystick(msg)
        self.get_logger().info(str(axis_values))

        # Publish the result
        self.publish_twist(axis_values, button_values)


    def parse_joystick(self, msg: Joy):
        axis_values = {}
        button_values = {}

        cfg_list = self.joy_map.get("joystick", [])
        if not isinstance(cfg_list, list):
            return {}, {}  # config malformed

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

            # Otherwise: ignore unknown entry types

        # If you want buttons too, return both (or store button_values on self)
        # For now, just store on self so you can use it later:
        self.last_buttons = button_values

        return axis_values, button_values

    def publish_twist(self, axis_values, button_values):
        """Publish the twist velocity command."""
        msg = Twist()
        msg.angular.y = axis_values['pitch']
        msg.angular.z = axis_values['yaw']
        msg.angular.x = button_values['roll_right'] - button_values['roll_left']

        msg.linear.x = axis_values['drive_forward']
        msg.linear.y = axis_values['strafe']
        msg.linear.z = axis_values['up'] - axis_values['down']

        self.twist_pub.publish(msg)


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