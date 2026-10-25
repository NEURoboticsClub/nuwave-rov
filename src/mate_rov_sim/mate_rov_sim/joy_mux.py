#!/usr/bin/env python3
"""
joy_mux: lets the keyboard and the gamepads share houston without fighting.

For each channel (thruster, arm) it republishes ONE Joy stream for houston:
  * keyboard, while keyboard_teleop is publishing (a key is held)
  * otherwise the gamepad, passed through untouched
  * otherwise neutral (all zeros = stop), so houston never acts on stale input

Subscribes:  /joy_thruster, /joy_arm                     (joy_node, the gamepads)
             /keyboard/joy_thruster, /keyboard/joy_arm    (keyboard_teleop)
Publishes:   /teleop/joy_thruster, /teleop/joy_arm        (houston reads these in the sim)
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class Channel:
    def __init__(self, node, name, gamepad_topic, keyboard_topic, output_topic):
        self.node = node
        self.name = name
        self.latest = {'gamepad': None, 'keyboard': None}
        self.stamp = {'gamepad': 0.0, 'keyboard': 0.0}
        self.source = None
        node.create_subscription(Joy, gamepad_topic, lambda m: self.on_msg('gamepad', m), 10)
        node.create_subscription(Joy, keyboard_topic, lambda m: self.on_msg('keyboard', m), 10)
        self.pub = node.create_publisher(Joy, output_topic, 10)

    def on_msg(self, source, msg):
        self.latest[source] = msg
        self.stamp[source] = time.monotonic()

    def select(self, now):
        if now - self.stamp['keyboard'] < self.node.keyboard_timeout:
            return 'keyboard'
        if now - self.stamp['gamepad'] < self.node.gamepad_timeout:
            return 'gamepad'
        return 'neutral'

    def publish(self, now):
        source = self.select(now)
        if source != self.source:
            self.node.get_logger().info(f'{self.name}: using {source}')
            self.source = source
        if source == 'neutral':
            ref = self.latest['gamepad'] or self.latest['keyboard']
            msg = Joy()
            msg.axes = [0.0] * (len(ref.axes) if ref else 8)
            msg.buttons = [0] * (len(ref.buttons) if ref else 11)
        else:
            msg = self.latest[source]
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub.publish(msg)


class JoyMux(Node):
    def __init__(self):
        super().__init__('joy_mux')
        self.declare_parameter('gamepad_thruster_topic', '/joy_thruster')
        self.declare_parameter('gamepad_arm_topic', '/joy_arm')
        self.declare_parameter('keyboard_thruster_topic', '/keyboard/joy_thruster')
        self.declare_parameter('keyboard_arm_topic', '/keyboard/joy_arm')
        self.declare_parameter('output_thruster_topic', '/teleop/joy_thruster')
        self.declare_parameter('output_arm_topic', '/teleop/joy_arm')
        self.declare_parameter('keyboard_timeout', 0.25)   # s since the last keyboard message
        self.declare_parameter('gamepad_timeout', 0.5)     # s since the last gamepad message
        self.declare_parameter('publish_rate', 50.0)

        gp = self.get_parameter
        self.keyboard_timeout = gp('keyboard_timeout').value
        self.gamepad_timeout = gp('gamepad_timeout').value
        self.channels = [
            Channel(self, 'thruster', gp('gamepad_thruster_topic').value,
                    gp('keyboard_thruster_topic').value, gp('output_thruster_topic').value),
            Channel(self, 'arm', gp('gamepad_arm_topic').value,
                    gp('keyboard_arm_topic').value, gp('output_arm_topic').value),
        ]
        self.create_timer(1.0 / gp('publish_rate').value, self.on_timer)

    def on_timer(self):
        now = time.monotonic()
        for channel in self.channels:
            channel.publish(now)


def main():
    rclpy.init()
    node = JoyMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
