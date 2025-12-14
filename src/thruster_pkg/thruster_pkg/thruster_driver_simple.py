#!/usr/bin/env python3
import sys
import time
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .PCA9685 import PCA9685

# ---- CONFIG ----
LOW_ANGLE = 0
HIGH_ANGLE = 180
STOP_ANGLE = 100


def arm_thruster(pwm, channel: int):
	"""Simple arming sequence and set to neutral/stop."""
	print("Arming thruster on channel", channel)
	try:
		pwm.setRotationAngle(channel, STOP_ANGLE)
	except Exception:
		try:
			pwm.set_rotation_angle(channel, STOP_ANGLE)
		except Exception:
			pass
	print(f" -> {STOP_ANGLE} (stop / neutral)")
	time.sleep(1.5)


class SimpleThrusterNode(Node):
	def __init__(self, channel_arg: int = None, topic_arg: str = None):
		# We'll declare parameters so this node can be configured via a launch
		# file. Use CLI args as fallbacks.
		super().__init__('thruster_simple')

		# Declare parameters with reasonable defaults (channel default 0)
		self.declare_parameter('channel', int(channel_arg) if channel_arg is not None else 0)

		# Read channel param 
		self.channel = int(self.get_parameter('channel').value)

		# Now declare topic param, defaulting to controller naming
		default_topic = topic_arg if topic_arg is not None else f'thruster/thruster_{self.channel}'
		self.declare_parameter('topic', default_topic)
		self.topic = str(self.get_parameter('topic').value)

		# update node name for clarity
		try:
			# rclpy doesn't support renaming a node after creation, so just log a clear message
			self.get_logger().info(f'Initializing thruster_simple node for channel {self.channel} on topic "{self.topic}"')
		except Exception:
			pass

		# Initialize PCA9685
		print("Initializing PCA9685...")
		self.pwm = PCA9685(bus=7)
		# prefer setPWMFreq API from your working script, fall back gracefully
		try:
			self.pwm.setPWMFreq(50)
		except Exception:
			try:
				self.pwm.set_pwm_freq(50)
			except Exception:
				pass
		print("PCA9685 initialized at ~50 Hz")

		# Arm thruster (send neutral)
		arm_thruster(self.pwm, self.channel)

		# Subscribe to topic
		self.sub = self.create_subscription(Float32, self.topic, self.cmd_cb, 10)
		print(f'Listening on "{self.topic}" for Float32 commands in [-1,1]')

	def cmd_cb(self, msg: Float32):
		cmd = float(msg.data)
		# Map normalized command [-1,1] to angle [LOW_ANGLE..HIGH_ANGLE]
		if cmd > 1.0:
			# assume user might send direct angle if >1
			angle = max(LOW_ANGLE, min(HIGH_ANGLE, int(cmd)))
		elif cmd < -1.0:
			# if very negative, ignore and clamp
			angle = LOW_ANGLE
		else:
			if cmd >= 0.0:
				angle = STOP_ANGLE + cmd * (HIGH_ANGLE - STOP_ANGLE)
			else:
				angle = STOP_ANGLE + cmd * (STOP_ANGLE - LOW_ANGLE)

		angle = int(max(LOW_ANGLE, min(HIGH_ANGLE, angle)))

		try:
			self.pwm.setRotationAngle(self.channel, angle)
		except Exception:
			try:
				self.pwm.set_rotation_angle(self.channel, angle)
			except Exception as e:
				self.get_logger().error(f'Failed to send angle: {e}')

	def destroy_node(self):
		# send neutral and cleanup
		try:
			try:
				self.pwm.setRotationAngle(self.channel, STOP_ANGLE)
			except Exception:
				try:
					self.pwm.set_rotation_angle(self.channel, STOP_ANGLE)
				except Exception:
					pass
			try:
				self.pwm.exit_PCA9685()
			except Exception:
				pass
		except Exception:
			pass

		if GPIO is not None:
			try:
				GPIO.cleanup()
			except Exception:
				pass

		super().destroy_node()


def main(argv=None):
	# if argv is None:
	# 	argv = sys.argv[1:]

	# if len(argv) < 1:
	# 	print("Usage: python3 thruster_driver_simple.py <channel> [topic]")
	# 	sys.exit(1)

	channel = int(7)
	topic = f'thruster/thruster_{channel}'

	rclpy.init()
	node = SimpleThrusterNode(channel, topic)

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print("KeyboardInterrupt — shutting down thruster node.")
	finally:
		node.get_logger().info('Shutting down thruster node...')
		try:
			node.destroy_node()
		except Exception:
			pass
		rclpy.shutdown()


if __name__ == '__main__':
	main()

