#!/usr/bin/env python3
"""
keyboard_teleop: drive the ROV and the arm from the keyboard.

Opens a small window (keys only register while it has focus) and publishes
gamepad-style sensor_msgs/Joy messages, built from houston's own
joystick_config.yaml, so key presses go through houston exactly like the
gamepads do: precision mode, stabilize and expo toggles, watchdogs and the
arm all behave the same. joy_mux merges these with the real gamepads.

  ROV                        ARM
    W / S   forward / back     J / L   base yaw left / right
    A / D   strafe left/right  I / K   shoulder up / down
    Q / E   yaw left / right   U / O   elbow up / down
    R / F   up / down          N / M   wrist rotate
    Up/Down pitch nose down/up Y / H   wrist pitch up / down
    Left/Right roll            [ / ]   claw open / close
  P precision mode   X stabilize   Z expo   (toggles, same as the gamepad buttons)

Publishes (only while a key is held, plus a short burst of neutral after release):
  thruster_topic (default /keyboard/joy_thruster)  sensor_msgs/Joy
  arm_topic      (default /keyboard/joy_arm)       sensor_msgs/Joy
"""
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nuwave_utils_pkg.file_helpers import load_yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# key -> list of (houston control name, value). Axis values are what houston should
# read AFTER its invert flag; buttons are 1 while held. houston computes
# linear.z = (up - down) * 0.5 and the wrist = (wrist_up - wrist_down) * 0.5,
# hence the +1/-1 pairs for full authority.
THRUSTER_KEYS = {
    'w': [('drive_forward', 1.0)],
    's': [('drive_forward', -1.0)],
    'a': [('strafe', -1.0)],
    'd': [('strafe', 1.0)],
    'q': [('yaw', 1.0)],
    'e': [('yaw', -1.0)],
    'r': [('up', 1.0), ('down', -1.0)],
    'f': [('up', -1.0), ('down', 1.0)],
    'Up': [('pitch', -1.0)],      # like pushing the pitch stick forward: nose down
    'Down': [('pitch', 1.0)],
    'Left': [('roll_left', 1)],
    'Right': [('roll_right', 1)],
    'p': [('precision_toggle', 1)],
    'x': [('stabilize_toggle', 1)],
    'z': [('expo_toggle', 1)],
}
ARM_KEYS = {
    'j': [('base_yaw_input', 1.0)],
    'l': [('base_yaw_input', -1.0)],
    'i': [('base_pitch_input', 1.0)],
    'k': [('base_pitch_input', -1.0)],
    'u': [('elbow_pitch_input', 1.0)],
    'o': [('elbow_pitch_input', -1.0)],
    'n': [('wrist_yaw_input', 1.0)],
    'm': [('wrist_yaw_input', -1.0)],
    'y': [('wrist_up_input', 1.0), ('wrist_down_input', -1.0)],
    'h': [('wrist_up_input', -1.0), ('wrist_down_input', 1.0)],
    'bracketleft': [('claw_open_input', 1)],
    'bracketright': [('claw_close_input', 1)],
}

HELP = """ROV                              ARM
  W / S      forward / back          J / L   base yaw left / right
  A / D      strafe left / right     I / K   shoulder up / down
  Q / E      yaw left / right        U / O   elbow up / down
  R / F      up / down               N / M   wrist rotate
  Up / Down  pitch nose down / up    Y / H   wrist pitch up / down
  Left/Right roll left / right       [ / ]   claw open / close

P precision mode    X stabilize    Z expo
Keys only work while this window has focus. A held key overrides the gamepad."""


class JoyLayout:
    """Where each houston control lives in a Joy message, from one section of joystick_config.yaml."""

    def __init__(self, entries):
        self.axes, self.buttons = {}, {}
        for cfg in entries or []:
            if 'axis' in cfg:
                self.axes[cfg['axis']] = (int(cfg.get('input', 0)), bool(cfg.get('invert', False)))
            elif 'button' in cfg:
                self.buttons[cfg['button']] = (int(cfg.get('input', 0)), bool(cfg.get('invert', False)))
        self.n_axes = max([i for i, _ in self.axes.values()] + [7]) + 1
        self.n_buttons = max([i for i, _ in self.buttons.values()] + [10]) + 1

    def missing(self, keymap):
        names = {name for pairs in keymap.values() for name, _ in pairs}
        return sorted(n for n in names if n not in self.axes and n not in self.buttons)

    def build(self, held, keymap):
        """Joy message for the set of held keys."""
        values = {}
        for key in held:
            for name, value in keymap.get(key, []):
                values[name] = values.get(name, 0) + value
        msg = Joy()
        msg.axes = [0.0] * self.n_axes
        msg.buttons = [0] * self.n_buttons
        for name, (index, invert) in self.axes.items():
            v = max(-1.0, min(1.0, float(values.get(name, 0.0))))
            msg.axes[index] = -v if invert else v
        for name, (index, invert) in self.buttons.items():
            pressed = 1 if values.get(name, 0) > 0 else 0
            # Several controls can share a button index (A and stabilize_toggle): any press wins
            msg.buttons[index] = max(msg.buttons[index], (1 - pressed) if invert else pressed)
        return msg


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('joy_config', '')   # '' = houston_pkg/config/joystick_config.yaml
        self.declare_parameter('thruster_topic', '/keyboard/joy_thruster')
        self.declare_parameter('arm_topic', '/keyboard/joy_arm')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('release_burst', 0.3)   # s of neutral after the last key is released

        gp = self.get_parameter
        path = gp('joy_config').value or os.path.join(
            get_package_share_directory('houston_pkg'), 'config', 'joystick_config.yaml')
        self.get_logger().info(f'Loading joystick config from: {path}')
        config = load_yaml(path) or {}
        self.thruster_layout = JoyLayout(config.get('thruster_control'))
        self.arm_layout = JoyLayout(config.get('arm_control'))
        for layout, keymap, section in ((self.thruster_layout, THRUSTER_KEYS, 'thruster_control'),
                                        (self.arm_layout, ARM_KEYS, 'arm_control')):
            for name in layout.missing(keymap):
                self.get_logger().warn(f'{section} has no "{name}" entry; its keys do nothing')

        self.thruster_pub = self.create_publisher(Joy, gp('thruster_topic').value, 10)
        self.arm_pub = self.create_publisher(Joy, gp('arm_topic').value, 10)
        self.period = 1.0 / gp('publish_rate').value
        self.release_burst = gp('release_burst').value

        self.modes = {}
        qos_state = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                               depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for mode in ('precision_mode', 'stabilize_enabled', 'expo_enabled'):
            self.create_subscription(Bool, f'/controls/{mode}',
                                     lambda msg, m=mode: self.modes.__setitem__(m, msg.data), qos_state)

        self.lock = threading.Lock()
        self.held = set()
        self.last_active = {'thruster': 0.0, 'arm': 0.0}

    def set_held(self, held):
        with self.lock:
            self.held = set(held)

    def publish(self):
        with self.lock:
            held = set(self.held)
        now = time.monotonic()
        for name, layout, keymap, pub in (('thruster', self.thruster_layout, THRUSTER_KEYS, self.thruster_pub),
                                          ('arm', self.arm_layout, ARM_KEYS, self.arm_pub)):
            mine = held & keymap.keys()
            if mine:
                self.last_active[name] = now
            elif now - self.last_active[name] > self.release_burst:
                continue  # idle: stay quiet so joy_mux passes the gamepad through
            msg = layout.build(mine, keymap)
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)


class KeyboardWindow:
    """Tk window that tracks which keys are physically held down."""

    # X11 auto-repeat sends release+press pairs while a key is held; a release only
    # counts if no new press follows within this many ms.
    RELEASE_DEBOUNCE_MS = 60

    def __init__(self, node):
        import tkinter as tk
        self.node = node
        self.held = set()
        self.pending_release = {}
        self.root = tk.Tk()
        self.root.title('ROV keyboard teleop')
        self.root.configure(padx=12, pady=10)
        tk.Label(self.root, text=HELP, justify='left', font=('TkFixedFont', 10)).pack(anchor='w')
        self.status = tk.Label(self.root, text='', justify='left', font=('TkFixedFont', 10, 'bold'))
        self.status.pack(anchor='w', pady=(8, 0))
        self.root.bind('<KeyPress>', self.on_press)
        self.root.bind('<KeyRelease>', self.on_release)
        self.root.bind('<FocusOut>', lambda _e: self.release_all())
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.root.after(int(node.period * 1000), self.tick)

    @staticmethod
    def key_of(event):
        k = event.keysym
        return k.lower() if len(k) == 1 else k   # caps lock / shift: 'W' -> 'w'

    def on_press(self, event):
        k = self.key_of(event)
        if k in self.pending_release:
            self.root.after_cancel(self.pending_release.pop(k))
        self.held.add(k)

    def on_release(self, event):
        k = self.key_of(event)
        if k in self.pending_release:
            self.root.after_cancel(self.pending_release[k])
        self.pending_release[k] = self.root.after(self.RELEASE_DEBOUNCE_MS, self._release, k)

    def _release(self, k):
        self.pending_release.pop(k, None)
        self.held.discard(k)

    def release_all(self):
        for job in self.pending_release.values():
            self.root.after_cancel(job)
        self.pending_release.clear()
        self.held.clear()

    def tick(self):
        self.node.set_held(self.held)
        self.node.publish()
        modes = self.node.modes
        on = [label for key, label in (('precision_mode', 'PRECISION'), ('stabilize_enabled', 'STABILIZE'),
                                       ('expo_enabled', 'EXPO')) if modes.get(key)]
        keys = sorted(self.held & (THRUSTER_KEYS.keys() | ARM_KEYS.keys()))
        self.status.config(text=f'Modes: {" ".join(on) or "-"}\nHeld:  {" ".join(keys) or "-"}')
        self.root.after(int(self.node.period * 1000), self.tick)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = KeyboardTeleop()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        window = KeyboardWindow(node)
    except Exception as e:  # no display (headless / SSH without X forwarding)
        node.get_logger().error(f'Cannot open the keyboard window ({e}). Is DISPLAY set? '
                                'Launch with keyboard:=false to skip it.')
    else:
        try:
            window.run()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
