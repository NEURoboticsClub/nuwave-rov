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
from .houston_functions import scale_controller_input, parse_joystick, publish_expo_state,  publish_precision_mode_state,  publish_stabilize_state, joy_arm_callback, publish_arm_commands, publish_twist_get_msg, _scale_to_unit

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
         self.expo_mode_pub.publish(publish_expo_state(self.expo_enabled))

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
        self.precision_mode_pub.publish(publish_precision_mode_state(self.precision_mode_enabled))

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
        self.stabilize_mode_pub.publish(publish_stabilize_state(self.stabilize_enabled))

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
        if self.last_joy_thruster_msg is None:
            return

        # Watchdog to prevent runaway if houston stops reveiving joystick messages
        now = self.get_clock().now()
        joy_stale = (
            self.last_joy_thruster_time is not None
            and (now - self.last_joy_thruster_time).nanoseconds * 1e-9 > self.joy_thruster_timeout
        )
        if joy_stale:
            if not self.joy_thruster_stale:
                self.get_logger().warn(
                    "Thruster joystick input stale; publishing neutral twist"
                )
                self.joy_thruster_stale = True
            self.twist_pub.publish(Twist())
            return
        if self.joy_thruster_stale:
            self.get_logger().info("Thruster joystick input restored")
            self.joy_thruster_stale = False

        try:
            parsed = self.parse_joystick(self.last_joy_thruster_msg, cfg_type="thruster_control")
            # self.get_logger().info(str(parsed))

            # Publish the result
            self.publish_twist(parsed["axis"], parsed["button"])
        except Exception as e:
            self.get_logger().error(f"Error processing thruster joystick input: {e}")

    def parse_joystick(self, msg: Joy, cfg_type: str) -> dict:
        cfg_list = self.joy_map.get(cfg_type, [])

        return parse_joystick(cfg_list, msg, self.expo_enabled)


    def publish_twist(self, axis_values, button_values):
        """Publish the twist velocity command."""
        stab_btn = int(button_values.get('stabilize_toggle', 0))
        if stab_btn == 1 and self.prev_stabilize_button == 0:
            self.set_stabilize_enabled(not self.stabilize_enabled, 'controller')
        self.prev_stabilize_button = stab_btn
    
        expo_btn = int(button_values.get('expo_toggle', 0))
        if expo_btn == 1 and self.prev_expo_toggle_button == 0:
            self.set_expo_enabled(not self.expo_enabled, 'controller')
        self.prev_expo_toggle_button = expo_btn
    
        precision_btn = int(button_values.get('precision_toggle', 0))
        if precision_btn == 1 and self.prev_precision_toggle_button == 0:
            self.set_precision_mode_enabled(not self.precision_mode_enabled, 'controller')
        self.prev_precision_toggle_button = precision_btn
    
        msg = publish_twist_get_msg(axis_values, button_values)
    
        if self.stabilize_enabled:
            now = self.get_clock().now()
            stale = (
                self.last_stabilizer_time is not None
                and (now - self.last_stabilizer_time).nanoseconds * 1e-9 > self.stabilizer_timeout
            )
            if stale:
                self.set_stabilize_enabled(False, 'stale_data')
                self.get_logger().warn(
                    "Stabilizer messages stale; stabilization disabled"
                )
            elif self.last_stabilizer_time is not None:
                s = self.last_stabilizer_twist
                lx, ly, lz = _scale_to_unit(
                    msg.linear.x + s.linear.x,
                    msg.linear.y + s.linear.y,
                    msg.linear.z + s.linear.z,
                )
                ax, ay, az = _scale_to_unit(
                    msg.angular.x + s.angular.x,
                    msg.angular.y + s.angular.y,
                    msg.angular.z + s.angular.z,
                )
                msg.linear.x, msg.linear.y, msg.linear.z = lx, ly, lz
                msg.angular.x, msg.angular.y, msg.angular.z = ax, ay, az
    
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
