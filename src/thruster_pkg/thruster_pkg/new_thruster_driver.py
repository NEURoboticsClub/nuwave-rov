import time
import math
import os
import rclpy
import sys
import uuid
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.duration import Duration

from .PCA9685 import PCA9685

class ThrusterNode(Node):
    """
    Subscribes: std_msgs/Float32 representing PWM signals 
    Safety: clamp, watchdog timeout -> neutral, optional slew limiting
    """

    def __init__(self, node_name : str):
        super().__init__(node_name)

        # Params
        self.declare_parameter('topic', '')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('channel', 0)
        self.declare_parameter('pwm_freq_hz', 50)             # ESC expects ~50 Hz
        self.declare_parameter('neutral_us', 1500.0)          # stop
        self.declare_parameter('min_us', 1000.0)              # full reverse (or forward, see invert)
        self.declare_parameter('max_us', 2000.0)              # full forward (or reverse)
        self.declare_parameter('deadband_us', 0.0)            # optional neutral deadband half-width
        self.declare_parameter('invert_direction', False)     # flip sign if needed
        self.declare_parameter('max_abs_cmd', 0.8)            # safety clamp
        self.declare_parameter('update_rate_hz', 50.0)        # write rate to the hat
        self.declare_parameter('watchdog_timeout_s', 0.5)     # neutral if no msg in this time
        self.declare_parameter('slew_us_per_s', 4000.0)       # 0 to disable rate limit; else max change per sec
        # Read Params
        topic = self.get_parameter('topic').get_parameter_value().string_value
        bus = self.get_parameter('i2c_bus').value
        addr = self.get_parameter('i2c_address').value
        self.channel = self.get_parameter('channel').value
        self.pwm_freq = self.get_parameter('pwm_freq_hz').value
        
        self.neutral_us = self.get_parameter('neutral_us').value
        self.min_us = self.get_parameter('min_us').value
        self.max_us = self.get_parameter('max_us').value

        self.deadband_us = self.get_parameter('deadband_us')
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.watchdog_timeout = Duration(
                seconds=self.get_parameter('watchdog_timeout_s').value
                )
        self.slew_us_per_s = self.get_parameter('slew_us_per_s').value)
        # Derived Vals
        self.period_us = 1_000_000.0 / self.pwm_freq
        if not topic: 
            # try to extract trailing _<number>
            import re
            m = re.search(r'(.+?)_(\d+)$', node_name)
            if m:
                topic = f'thruster/{m.group(1)}_{m.group(2)}'
            else:
                topic = 'thruster/thruster_0'
        elif not topic.startswith('/'):
            topic = '/' + topic

        # State
        self.target_us = self.neutral_us
        self.current_us = self.neutral_us
        self.last_msg_time = self.get_clock().now()

        # PCA stuff
        try:
            self.pwm = PCA9685(bus=bus, address=addr)
            self.pwm.setPWMFreq(int(self.pwm_freq))
            self._write_us(self.neutral_us)
            self.get_logger().info("Def put something here")
        except Exception as e:
            self.pwm = None
            self.get_logger().error(f'PCA9685 init failed: {e}')

        # Ros Wiring
        self.sub = self.create_subscription(Float32, topic, self.cmd_callback, 10)
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.update)

        self.get_logger().info(
                f'Listening on "{topic}" for Float32 (us), '
                f'range [{self.min_us, self.max_us}], neutral_us={self.neutral_us}'
                )

        
    def _write_us(self, us : float):
        """
        Clamp, convert and write to hardware.
        """
        us = max(self.min_us, min(self.max_us, us))
        off_ticks = int(us * 4096 / self.period_us)
        off_ticks = max(0, min(4095, off_ticks))
        self.pwm.setPWM(self.channel, 0, off_ticks)

    # Callbacks
    def cmd_callback(self, msg : Float32):
        """
        Signal to SPIN BABY SPIN
        """
        self.target_us = max(self.min_us, min(self.max_us, msg.data))
        self.last_msg_time = self.get_clock().now()

    def update(self): 
        now = self.get_clock().now()
        dt = (now - getattr(self, "_prev_time", now)).nanoseconds / 1e9
        self._prev_time = now

