import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import yaml
import os
import numpy as np

class Houston(Node):
    def __init__(self):
        super().__init__('thruster_controller')

        # === Parameters ===
        self.declare_parameter('joy_config', '/home/nuwave/nuwave-rov/install/controller/share/controller/config/joystick_config.yaml')
        self.declare_parameter('joy_topic', '/joy')

        joy_config_path = self.get_parameter('joy_config').value
        joy_topic = self.get_parameter('joy_topic').value

        self.get_logger().info(f"Loading joystick config from: {joy_config_path}")
        # === Load configurations ===
        self.joy_map = self.load_yaml(joy_config_path)
        
        # === Subscribers / Publishers ===
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.thruster_pubs = [self.create_publisher(Float32, thruster_topic+'/thruster_'+str(i), 10) for i in range(len(self.thruster_map['thrusters']))]
        
        # === Internal state ===
        self.last_joy_msg = None
        
        # === Compute Thruster Allocation Matrix ===
        self.A = self.compute_thruster_allocation_matrix()
        self.A_pinv = np.linalg.pinv(self.A)  # Compute the pseudoinverse of A

        self.get_logger().info("Thruster Controller Initialized")

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
        thruster_outputs = self.compute_thrusters(axis_values, button_values)

        # Publish the result
        self.publish_thrusters(thruster_outputs)

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

    def publish_twist(self, twist):
        """Publish the computed thruster duty cycles."""
        # self.get_logger().info(str(thruster_outputs))
        for i in range(len(self.thruster_pubs)):
            msg = Float32()
            msg.data = twist[i]
            self.thruster_pubs[i].publish(msg)


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