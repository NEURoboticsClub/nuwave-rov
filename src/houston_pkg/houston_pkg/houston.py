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
        self.declare_parameter('publish_rate_hz', 50.0)    # publish rate of houston twist commands
        self.declare_parameter('expo_enabled_default', False)
        self.declare_parameter('agnes_enabled_default', False)
        self.declare_parameter('agnes_scale', 0.2)

        joy_config_path = self.get_parameter('joy_config').value
        joy_thruster = self.get_parameter('joy_thruster').value
        joy_arm = self.get_parameter('joy_arm').value
        rate = self.get_parameter('publish_rate_hz').value

        self.stabilizer_timeout = self.get_parameter('stabilizer_timeout').value
        self.last_stabilizer_time = None


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
        self.agnes_mode_pub = self.create_publisher(Bool, '/controls/agnes_enabled', qos_state)
        self.agnes_mode_sub = self.create_subscription(Bool, '/gui_buttons/agnes_enabled', self.gui_agnes_toggle_callback, 10)

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
        self.prev_agnes_toggle_button = 0
        self.expo_enabled = bool(self.get_parameter('expo_enabled_default').value)
        self.agnes_enabled = bool(self.get_parameter('agnes_enabled_default').value)
        self.agnes_scale = float(self.get_parameter('agnes_scale').value)

        self.create_timer(1.0 / rate, self._publish_loop)

        self.publish_expo_state()
        self.publish_agnes_state()

        self.get_logger().info("Houston Initialized")

    def publish_expo_state(self):
        msg = Bool()
        msg.data = bool(self.expo_enabled)
        self.expo_mode_pub.publish(msg)

    def set_expo_enabled(self, enabled: bool, source: str):
        enabled = bool(enabled)
        if self.expo_enabled == enabled:
            return
        self.expo_enabled = enabled
        self.publish_expo_state()
        self.get_logger().info(f"Exponential controls: {'ON' if self.expo_enabled else 'OFF'} (source={source})")

    def gui_expo_toggle_callback(self, msg: Bool):
        self.set_expo_enabled(msg.data, 'gui')

    def publish_agnes_state(self):
        msg = Bool()
        msg.data = bool(self.agnes_enabled)
        self.agnes_mode_pub.publish(msg)

    def set_agnes_enabled(self, enabled: bool, source: str):
        enabled = bool(enabled)
        if self.agnes_enabled == enabled:
            return
        self.agnes_enabled = enabled
        self.publish_agnes_state()
        self.get_logger().info(
            f"Agnes mode: {'ON' if self.agnes_enabled else 'OFF'} "
            f"(scale={self.agnes_scale:.2f}, source={source})"
        )

    def gui_agnes_toggle_callback(self, msg: Bool):
        self.set_agnes_enabled(msg.data, 'gui')

    def scale_controller_input(self, x: float) -> float:
        """Apply a normalized exponential joystick curve based on the requested shape."""
        # x is expected in [-1, 1]. Match existing deadband intent with a small center deadzone.
        if abs(x) <= 0.05:
            return 0.0

        abs_x = abs(x)
        result = np.sign(x) * ((1.2 * np.power(1.0356, abs_x * 100.0)) - 1.2 + (0.2 * abs_x * 100.0))

        # Normalize curve output back to [-1, 1].
        max_result = (1.2 * np.power(1.0356, 100.0)) - 1.2 + (0.2 * 100.0)
        if max_result <= 0:
            return float(x)
        return float(np.clip(result / max_result, -1.0, 1.0))

    def stabilizer_callback(self, msg: Twist):
        self.last_stabilizer_twist = msg
        self.last_stabilizer_time = self.get_clock().now()

    def joy_arm_callback(self, msg: Joy):
        """Handle arm joystick input"""
        try:
            self.last_joy_arm_msg = msg
            return_map = self.parse_joystick(msg, cfg_type="arm_control")
            self.get_logger().info(str(return_map))

            self.publish_arm_commands(return_map["axis"], return_map["button"])
        except Exception as e:
            self.get_logger().error(f"Error processing arm joystick input: {e}")

    def joy_thruster_callback(self, msg: Joy):
        """Handle thruster joystick input"""
        self.last_joy_thruster_msg = msg

    def _publish_loop(self):
        if self.last_joy_thruster_msg is None:
            return
        try:
            parsed = self.parse_joystick(self.last_joy_thruster_msg, cfg_type="thruster_control")
            self.get_logger().info(str(parsed))

            # Publish the result
            self.publish_twist(parsed["axis"], parsed["button"])
        except Exception as e:
            self.get_logger().error(f"Error processing thruster joystick input: {e}")

    def parse_joystick(self, msg: Joy, cfg_type: str) -> dict:
        cfg_list = self.joy_map.get(cfg_type, [])

        if not isinstance(cfg_list, list):
            return {"axis": {}, "button": {}} # config malformed

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
                # Controller is NOISEY! Tune deadzone, so that stick drift still gives us neutral when we have it at rest
                # Clamp below 1.0 so the rescaling can't divide by zero
                deadzone = min(max(float(cfg.get("deadzone", 0.07)), 0.0), 0.99)

                raw = msg.axes[axis_index] if axis_index < len(msg.axes) else 0.0
                if invert:
                    raw *= -1.0
                
                # Deadzone processing
                if abs(raw) < deadzone:
                    raw = 0.0
                else:
                    # Deadzone rescaling, so it is still within the [-1, 1] range
                    # This is so it doesn't go 0.0 to +- deadzone immediately.
                    raw = (raw - np.sign(raw) * deadzone) / (1.0 - deadzone)

                val = raw * sensitivity

                # Optional: support your "scale" field (keep simple)
                if scale == "logarithmic":
                    # compress near 0, preserve sign
                    val = np.sign(val) * np.log1p(abs(val))
                elif scale == "exponential" and self.expo_enabled:
                    val = self.scale_controller_input(val)

                val = float(np.clip(val, -1.0, 1.0))

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
        stab_btn = int(button_values.get('stabilize_toggle', 0))
        if stab_btn == 1 and self.prev_stabilize_button == 0:
            self.stabilize_enabled = not self.stabilize_enabled
            self.get_logger().info(f"Stabilization mode: {'ON' if self.stabilize_enabled else 'OFF'}")
            if self.stabilize_enabled:  # send message to capture orientation as setpoint
                self.capture_pub.publish(Empty())
        self.prev_stabilize_button = stab_btn

        expo_btn = int(button_values.get('expo_toggle', 0))
        if expo_btn == 1 and self.prev_expo_toggle_button == 0:
            self.set_expo_enabled(not self.expo_enabled, 'controller')
        self.prev_expo_toggle_button = expo_btn

        agnes_btn = int(button_values.get('agnes_toggle', 0))
        if agnes_btn == 1 and self.prev_agnes_toggle_button == 0:
            self.set_agnes_enabled(not self.agnes_enabled, 'controller')
        self.prev_agnes_toggle_button = agnes_btn

        msg = Twist()
        msg.angular.x = float(axis_values.get('pitch', 0.0))
        msg.angular.y = float(button_values.get('roll_right', 0.0)) - float(button_values.get('roll_left', 0.0))
        msg.angular.z = float(axis_values.get('yaw', 0.0))


        msg.linear.x = float(axis_values.get('strafe', 0.0))
        msg.linear.y = float(axis_values.get('drive_forward', 0.0))
        msg.linear.z = (float(axis_values.get('up', 0.0)) - float(axis_values.get('down', 0.0))) * 0.5

        def _scale_to_unit(x, y, z):
            m = max(abs(x), abs(y), abs(z), 1.0)
            return x / m, y / m, z / m

        if self.stabilize_enabled:
            now = self.get_clock().now()
            stale = (
                self.last_stabilizer_time is None
                or (now - self.last_stabilizer_time).nanoseconds * 1e-9 > self.stabilizer_timeout
            )
            if stale:
                self.stabilize_enabled = False
                self.get_logger().warn(
                    "Stabilizer messages stale; stabilization disabled"
                )
            else:
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

        if self.agnes_enabled:
            # Agnes mode limits only vertical thrust authority.
            msg.linear.z *= self.agnes_scale

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
            (float(axis_values.get('wrist_up_input', 0.0)) - float(axis_values.get('wrist_down_input', 0.0))) * 0.5,
            (float(button_values.get('claw_open_input', 0.0)) - float(button_values.get('claw_close_input', 0.0))), 
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
