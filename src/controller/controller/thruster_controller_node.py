import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import yaml
import os
import numpy as np

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')

        # === Parameters ===
        self.declare_parameter('joy_config', 'joystick_config.yaml')
        self.declare_parameter('thruster_config', 'thruster_config.yaml')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('thruster_topic', '/thruster')

        joy_config_path = self.get_parameter('joy_config').value
        thruster_config_path = self.get_parameter('thruster_config').value
        joy_topic = self.get_parameter('joy_topic').value
        thruster_topic = self.get_parameter('thruster_topic').value

        # === Load configurations ===
        self.joy_map = self.load_yaml(joy_config_path)
        self.thruster_map = self.load_yaml(thruster_config_path)

        # === Subscribers / Publishers ===
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.thruster_pubs = [self.create_publisher(Float32, thruster_topic+'/'+str(i), 10) for i in range(len(self.thruster_map))]
        
        # === Internal state ===
        self.last_joy_msg = None

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
        axis_values = self.parse_joystick(msg)

        thruster_outputs = self.compute_thrusters(axis_values)

        # Publish the result
        self.publish_thrusters(thruster_outputs)

    def parse_joystick(self, msg: Joy):
        """Extract and scale joystick inputs according to config."""
        axis_values = {}
        if "joystick" not in self.joy_map:
            return axis_values

        for axis_cfg in self.joy_map["joystick"]:
            axis_name = axis_cfg["axis"]
            axis_index = axis_cfg["input"]
            invert = axis_cfg.get("invert", False)
            sensitivity = axis_cfg.get("sensitivity", 1.0)

            value = msg.axes[axis_index] if axis_index < len(msg.axes) else 0.0
            if invert:
                value *= -1
            axis_values[axis_name] = value * sensitivity

        return axis_values

    def compute_thruster_allocation_matrix(self):
        pass
        # TODO


    def compute_thrusters(self, axis_values):
        """
        Compute the duty cycle (-1 to 1) for each thruster.
        Fill this with your own math later.
        """
        thruster_outputs = []
        for thruster in self.thruster_map.get("thrusters", []):
            # Placeholder — replace with your own math
            duty = 0.0
            thruster_outputs.append(duty)
        return thruster_outputs

    def publish_thrusters(self, thruster_outputs):
        """Publish the computed thruster duty cycles."""
        for i in range(len(self.thruster_pubs)):
            msg = Float32()
            msg.data = thruster_outputs[i]
            self.thruster_pubs[i].publish(msg)


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
