import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import yaml
import os
import time


class JoystickIdentify(Node):
    def __init__(self):
        super().__init__('joystick_identify')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('output_file', 'joystick_config.yaml')

        joy_topic = self.get_parameter('joy_topic').value
        self.output_file = self.get_parameter('output_file').value

        # Subscribe to joystick
        self.subscription = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.last_msg = None

        self.config = {"joystick": []}
        # strafe, drive_forward, yaw, pitch, up, down
        self.axis_names = ["strafe", "drive_forward", "yaw", "pitch", "up", "down"]

        self.get_logger().info("🎮 Joystick Identify Node Started")
        self.get_logger().info("Move your joystick and press buttons to identify controls.")
        self.get_logger().info("Press CTRL+C when ready to configure and save.")

    def joy_callback(self, msg: Joy):
        """Display live joystick activity for user reference."""
        self.last_msg = msg
        os.system('clear')

        print("=== Joystick Identify Tool ===")
        print("Move your joystick or press buttons to see which index changes.\n")

        print("Axes:")
        for i, val in enumerate(msg.axes):
            print(f"  Axis {i}: {val:+.2f}")

        print("\nButtons:")
        for i, val in enumerate(msg.buttons):
            state = "Pressed" if val else "Released"
            print(f"  Button {i}: {state}")

        print("\n(Press CTRL+C when done identifying indexes.)")
        time.sleep(0.1)

    def interactive_setup(self):
        """Wizard for setting up both axes and buttons."""
        if self.last_msg is None:
            print("⚠️ No joystick messages received yet. Move or press something first.")
            return

        print("\n=== Axis Configuration ===")
        for axis_name in self.axis_names:
            try:
                idx = int(input(f"\nEnter axis index for '{axis_name}': "))
                inv = input("Invert this axis? (y/n): ").strip().lower() == 'y'
                sens = float(input("Sensitivity (default=1.0): ") or 1.0)
                scale = input("Scale type [linear/logarithmic] (default=linear): ").strip() or "linear"
            except (ValueError, KeyboardInterrupt):
                print("\n❌ Cancelled or invalid input.")
                return

            self.config["joystick"].append({
                "axis": axis_name,
                "input": idx,
                "invert": inv,
                "sensitivity": sens,
                "scale": scale
            })

        print("\n=== Button Configuration ===")
        add_buttons = input("Do you want to configure buttons? (y/n): ").strip().lower() == 'y'
        while add_buttons:
            try:
                btn_name = input("Enter button name (e.g., A, B, Start): ").strip()
                btn_idx = int(input("Enter button index number: "))
                btn_inv = input("Invert this button? (y/n): ").strip().lower() == 'y'
            except (ValueError, KeyboardInterrupt):
                print("\n❌ Cancelled or invalid input.")
                break

            self.config["joystick"].append({
                "button": btn_name,
                "input": btn_idx,
                "invert": btn_inv
            })

            add_buttons = input("Add another button? (y/n): ").strip().lower() == 'y'

        self.save_config()

    def save_config(self):
        """Save configuration to YAML file."""
        with open(self.output_file, 'w') as f:
            yaml.dump(self.config, f, sort_keys=False)
        print(f"\n✅ Configuration saved to: {self.output_file}")
        print("\nGenerated config:\n")
        print(yaml.dump(self.config, sort_keys=False))

def main(args=None):
    rclpy.init(args=args)
    node = JoystickIdentify()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # After user stops the node, run interactive setup
    node.interactive_setup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
