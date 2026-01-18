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
        self.declare_parameter('joy_config', '/home/nuwave-rov/nuwave-rov/install/controller/share/controller/config/please_work.yaml')
        self.declare_parameter('thruster_config', '/home/nuwave-rov/nuwave-rov/install/controller/share/controller/config/thruster_config.yaml')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('thruster_topic', '/thruster')


        joy_config_path = self.get_parameter('joy_config').value
        thruster_config_path = self.get_parameter('thruster_config').value
        joy_topic = self.get_parameter('joy_topic').value
        thruster_topic = self.get_parameter('thruster_topic').value

        self.get_logger().info(f"Loading joystick config from: {joy_config_path}")
        self.get_logger().info(f"Loading thruster config from: {thruster_config_path}")
        # === Load configurations ===
        self.joy_map = self.load_yaml(joy_config_path)
        self.thruster_map = self.load_yaml(thruster_config_path)
        self.get_logger().info(str(self.thruster_map))
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
        axis_values = self.parse_joystick(msg)
        self.get_logger().info(str(axis_values))
        thruster_outputs = self.compute_thrusters(axis_values)

        # Publish the result
        self.publish_thrusters(thruster_outputs)

    def parse_joystick(self, msg: Joy):
        axis_values = {}
        button_values = {}

        cfg_list = self.joy_map.get("joystick", [])
        if not isinstance(cfg_list, list):
            return axis_values  # config malformed

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

        return axis_values

    def compute_thruster_allocation_matrix(self):
        """
        Builds the transformation matrix A that maps thruster outputs -> vehicle forces:
        [surge, sway, heave, yaw]^T = A @ [T1..T8]^T
        """
        thrusters = self.thruster_map.get("thrusters", [])
        A = np.zeros((4, len(thrusters)))  # 4 DOFs × N thrusters

        for i, thruster in enumerate(thrusters):
            angle_deg = thruster.get("angle_deg", 0.0)
            angle_rad = np.deg2rad(angle_deg)
            lever_arm = thruster.get("lever_arm_distance", 0.0)
            pos = thruster.get("position", "").lower()

            # Horizontal thrusters: contribute to surge, sway, and yaw
            if pos in ["front", "rear"]:
                A[0, i] = np.cos(angle_rad)      # Surge
                A[1, i] = np.sin(angle_rad)      # Sway

                # Yaw contribution (direction depends on front/rear and rotation sense)
                if pos == "front":
                    A[3, i] = np.sign(angle_deg) * lever_arm
                elif pos == "rear":
                    A[3, i] = -np.sign(angle_deg) * lever_arm

            # Vertical thrusters: contribute only to heave
            elif pos in ["up", "down"]:
                A[2, i] = np.sign(angle_deg)  # +1 for up, -1 for down

        self.get_logger().info(f"Thruster allocation matrix:\n{A}")
        return A


    def compute_thrusters(self, axis_values):
        """
        Map joystick input (surge, sway, heave, yaw) to 8 thruster commands.
        """
        U = np.array([
            axis_values.get("surge", 0.0),
            axis_values.get("sway", 0.0),
            axis_values.get("heave", 0.0),
            axis_values.get("yaw", 0.0)
        ])

        T = self.A_pinv @ U  # Thruster outputs

        # Clamp to [-1, 1]
        T = np.clip(T, -1.0, 1.0)

        return T.tolist()

    def publish_thrusters(self, thruster_outputs):
        """Publish the computed thruster duty cycles."""
        # self.get_logger().info(str(thruster_outputs))
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