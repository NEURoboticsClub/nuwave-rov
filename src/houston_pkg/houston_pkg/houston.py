import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Empty, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from nuwave_utils_pkg.file_helpers import load_yaml
import os
import numpy as np
from .houston_functions import scale_controller_input, parse_joystick, publish_boolean, joy_arm_callback, publish_arm_commands, publish_twist_get_msg, set_button_enabled, check_if_stale_and_set_msg, publish_loop, try_parse_joystick_publish

class Houston(Node):
    """Status node to map joytick inputs to Twist commands."""

    def __init__(self):
        super().__init__('houston')

        pkg_share = get_package_share_directory('houston_pkg')
        
        # === Parameters ===
        self.declare_parameter('joy_config', os.path.join(pkg_share, 'config', 'joystick_config.yaml'))
        self.declare_parameter('joy_thruster', '/joy_thruster')
        self.declare_parameter('joy_arm', '/joy_arm')
        self.declare_parameter('stabilizer_timeout', 0.5)  # seconds before we consider stabilizer data stale and disable stabilization
        self.declare_parameter('joy_thruster_timeout', 0.5)  # seconds before we consider thruster joystick data stale and zero the twist
        self.declare_parameter('publish_rate_hz', 50.0)    # publish rate of houston twist commands
        self.declare_parameter('expo_enabled_default', False)
        self.declare_parameter('precision_mode_default', False)

        joy_config_path = self.get_parameter('joy_config').value
        joy_thruster = self.get_parameter('joy_thruster').value
        joy_arm = self.get_parameter('joy_arm').value
        rate = self.get_parameter('publish_rate_hz').value

        self.stabilizer_timeout = self.get_parameter('stabilizer_timeout').value
        self.last_stabilizer_time = None

        self.joy_thruster_timeout = self.get_parameter('joy_thruster_timeout').value
        self.last_joy_thruster_time = None
        self.joy_thruster_stale = False


        self.get_logger().info(f"Loading joystick config from: {joy_config_path}")
        # === Load configurations ===
        self.joy_map = load_yaml(joy_config_path)
        
        # === Subscribers / Publishers ===
        self.thruster_joy_sub = self.create_subscription(Joy, joy_thruster, self.joy_thruster_callback, 10)
        self.arm_joy_sub = self.create_subscription(Joy, joy_arm, self.joy_arm_callback, 10)

        self.stabilizer_sub = self.create_subscription(Twist, '/stabilizer/commands', self.stabilizer_callback, 10)

        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.expo_mode_pub = self.create_publisher(Bool, '/controls/expo_enabled', qos_state)
        self.expo_mode_sub = self.create_subscription(Bool, '/gui_buttons/expo_enabled', self.gui_expo_toggle_callback, 10)
        self.precision_mode_pub = self.create_publisher(Bool, '/controls/precision_mode', qos_state)
        self.precision_mode_sub = self.create_subscription(Bool, '/gui_buttons/precision_mode', self.gui_precision_mode_toggle_callback, 10)
        self.stabilize_mode_pub = self.create_publisher(Bool, '/controls/stabilize_enabled', qos_state)
        self.stabilize_mode_sub = self.create_subscription(Bool, '/gui_buttons/stabilize_enabled', self.gui_stabilize_toggle_callback, 10)

        self.twist_pub = self.create_publisher(Twist, "velocity_commands", 10)
        self.arm_pub = self.create_publisher(Float32MultiArray, "arm_commands", 10)
        self.capture_pub = self.create_publisher(Empty, '/stabilizer/capture', 10)
        
        # === Internal state ===
        self.last_joy_thruster_msg = None
        self.last_joy_arm_msg = None
        
        self.last_stabilizer_twist = Twist()
        self.stabilize_enabled = False
        self.prev_stabilize_button = 0
        self.prev_expo_toggle_button = 0
        self.prev_precision_toggle_button = 0
        self.expo_enabled = bool(self.get_parameter('expo_enabled_default').value)
        self.precision_mode_enabled = bool(self.get_parameter('precision_mode_default').value)

        self.create_timer(1.0 / rate, self._publish_loop)

        self.publish_expo_state()
        self.publish_precision_mode_state()
        self.publish_stabilize_state()

        self.get_logger().info("Houston Initialized")

    def publish_expo_state(self):
         publish_boolean(self.expo_enabled, self.expo_mode_pub)

    def set_expo_enabled(self, enabled: bool, source: str):
        enabled = bool(enabled)
        if self.expo_enabled == enabled:
            return
        self.expo_enabled = enabled
        self.publish_expo_state()
        self.get_logger().info(f"Exponential controls: {'ON' if self.expo_enabled else 'OFF'} (source={source})")

    def gui_expo_toggle_callback(self, msg: Bool):
        self.set_expo_enabled(msg.data, 'gui')

    def publish_precision_mode_state(self):
        publish_boolean(self.precision_mode_enabled,self.precision_mode_pub)

    def set_precision_mode_enabled(self, enabled: bool, source: str):
        enabled = bool(enabled)
        if self.precision_mode_enabled == enabled:
            return
        self.precision_mode_enabled = enabled
        self.publish_precision_mode_state()
        self.get_logger().info(f"Precision mode: {'ON' if self.precision_mode_enabled else 'OFF'} (source={source})")

    def gui_precision_mode_toggle_callback(self, msg: Bool):
        self.set_precision_mode_enabled(msg.data, 'gui')

    def publish_stabilize_state(self):
        publish_boolean(self.stabilize_enabled, self.stabilize_mode_pub)

    def set_stabilize_enabled(self, enabled: bool, source: str):
        enabled = bool(enabled)
        if self.stabilize_enabled == enabled:
            return
        self.stabilize_enabled = enabled
        self.publish_stabilize_state()
        self.get_logger().info(f"Stabilization mode: {'ON' if self.stabilize_enabled else 'OFF'} (source={source})")
        if self.stabilize_enabled:
            self.capture_pub.publish(Empty())

    def gui_stabilize_toggle_callback(self, msg: Bool):
        self.set_stabilize_enabled(msg.data, 'gui')

    def scale_controller_input(self, x: float) -> float:
        """Apply a normalized exponential joystick curve based on the requested shape."""
        # x is expected in [-1, 1]. Match existing deadband intent with a small center deadzone.
        return scale_controller_input(x)

    def stabilizer_callback(self, msg: Twist):
        self.last_stabilizer_twist = msg
        self.last_stabilizer_time = self.get_clock().now()

    def joy_arm_callback(self, msg: Joy):
        """Handle arm joystick input"""
        try:
            self.last_joy_arm_msg = msg
            return_map = joy_arm_callback(msg)
            self.publish_arm_commands(return_map["axis"], return_map["button"])
        except Exception as e:
            self.get_logger().error(f"Error processing arm joystick input: {e}")

    def joy_thruster_callback(self, msg: Joy):
        """Handle thruster joystick input"""
        self.last_joy_thruster_msg = msg
        self.last_joy_thruster_time = self.get_clock().now()

    def _publish_loop(self):
       self.joy_thruster_stale = publish_loop(self.last_joy_thruster_msg, self.get_clock(), self.last_joy_thruster_time, self.joy_thruster_timeout, self.joy_thruster_stale, self.get_logger(), self.twist_pub)
       try_parse_joystick_publish(self.last_joy_thruster_msg, self.parse_joystick, self.publish_twist, self.get_logger())

    def parse_joystick(self, msg: Joy, cfg_type: str) -> dict:
        return parse_joystick(self.joy_map.get(cfg_type, []), msg, self.expo_enabled)


    def publish_twist(self, axis_values, button_values):
        """Publish the twist velocity command."""
        set_button_enabled(button_values, 'stabilize_toggle', self.prev_stabilize_button, self.set_stabilize_enabled, self.stabilize_enabled)
        
        set_button_enabled(button_values, "expo_toggle", self.prev_expo_toggle_button, self.set_expo_enabled, self.expo_enabled)
    
        set_button_enabled(button_values, 'precision_toggle', self.prev_precision_toggle_button, self.precision_mode_enabled, self.precision_mode_enabled)
    
        msg = publish_twist_get_msg(axis_values, button_values)
    
        if self.stabilize_enabled:
            check_if_stale_and_set_msg(self.get_clock(), self.last_stabilizer_time, self.last_stabilizer_twist, self.stabilizer_timeout, self.set_stabilize_enabled, self.get_logger(), msg)
    
        self.twist_pub.publish(msg)


    def publish_arm_commands(self, axis_values, button_values):
        """Publish arm control commands."""
        # 6 element array: [axis1 - axis6]
        # Create and publish arm commands as a 6-element array
        
        
        self.arm_pub.publish(publish_arm_commands(axis_values, button_values))



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
