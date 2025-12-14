import time
import math
import os
import sys
import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.duration import Duration

# Use the external PCA9685 implementation (the user's working script uses
# `from PCA9685 import PCA9685` with methods setPWMFreq, setRotationAngle,
# and exit_PCA9685).
from PCA9685 import PCA9685

class ThrusterNode(Node):
    """
    Subscribes: std_msgs/Float32 on <topic> (default: /cmd_thrust), range [-1, 1].
    Maps to PWM us around neutral and writes to PCA9685 channel.
    Safety: clamp, watchdog timeout -> neutral, optional slew limiting.
    """

    def __init__(self, node_name: str = None):
        # Allow creating multiple instances by giving each a distinct name.
        # If none provided, generate a short unique name suffix.
        if node_name is None:
            node_name = f'thruster_node_{uuid.uuid4().hex[:8]}'
        super().__init__(node_name)

        # If empty, derive topic from node name
        self.declare_parameter('topic', '')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x40)           # PCA9685 default
        self.declare_parameter('channel', 0)                  # PCA9685 output channel
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

        topic = self.get_parameter('topic').get_parameter_value().string_value

        # Derive topic from node_name when topic param not provided. Expect
        # node_name like 'thruster_0' or base 'thruster_<n>'. If derivation
        # fails, fall back to '/thruster_0'. Ensure topic starts with '/'.
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

        bus = int(self.get_parameter('i2c_bus').value)
        addr = int(self.get_parameter('i2c_address').value)
        self.channel = int(self.get_parameter('channel').value)
        pwm_freq = float(self.get_parameter('pwm_freq_hz').value)

        self.neutral_us = float(self.get_parameter('neutral_us').value)
        self.min_us = float(self.get_parameter('min_us').value)
        self.max_us = float(self.get_parameter('max_us').value)
        self.deadband_us = float(self.get_parameter('deadband_us').value)
        self.invert = bool(self.get_parameter('invert_direction').value)
        self.max_abs_cmd = float(self.get_parameter('max_abs_cmd').value)
        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.watchdog_timeout = Duration(seconds=float(self.get_parameter('watchdog_timeout_s').value))
        self.slew_us_per_s = float(self.get_parameter('slew_us_per_s').value)

        # Derived values
        self.period_us = int(1_000_000.0 / pwm_freq)
        self.last_msg_time = self.get_clock().now()
        self.current_cmd = 0.0
        self.current_us = self.neutral_us  # track last output for slew limiting
        # Init hardware using the external PCA9685 class.
        # The external class from the user's script expects `bus=` and then
        # calling `setPWMFreq(freq)`; it exposes `setRotationAngle(channel, angle)`
        # for servo-like output and `exit_PCA9685()` for cleanup.
        try:
            self.pwm = PCA9685(bus=bus)
            # configure frequency like the user's working script
            try:
                self.pwm.setPWMFreq(int(pwm_freq))
            except Exception:
                # Some PCA implementations use set_pwm_freq style; ignore if not present
                try:
                    self.pwm.set_pwm_freq(int(pwm_freq))
                except Exception:
                    pass
            self.get_logger().info(f'PCA9685 on bus {bus}, addr 0x{addr:02X}, freq {pwm_freq} Hz')

            # Send neutral signal immediately on startup to initialize ESCs.
            neutral_angle = int(self.us_to_angle(self.neutral_us))
            self.pwm.setRotationAngle(self.channel, neutral_angle)
            self.get_logger().info(f'Sent neutral ({neutral_angle} deg) to channel {self.channel} on init')
        except Exception as e:
            # Log and continue; periodic updates will keep trying
            self.get_logger().warning(f'Failed to initialize PCA9685: {e}')

        # ROS wiring: subscribe to controller outputs (Float32 per-thruster)
        self.sub = self.create_subscription(Float32, topic, self.cmd_cb, 10)
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.update_output)

        self.get_logger().info(
            f'Listening on "{topic}" for Float32 in [-1,1]; driving channel {self.channel}'
        )

    def cmd_cb(self, msg: Float32):
        cmd = max(-self.max_abs_cmd, min(self.max_abs_cmd, msg.data))
        if self.invert:
            cmd = -cmd
        self.current_cmd = cmd
        self.last_msg_time = self.get_clock().now()

    def map_cmd_to_us(self, cmd: float) -> float:
        """
        Linear map from [-1, 1] to [min_us, max_us] around neutral,
        respecting optional neutral deadband.
        """
        # cmd=-1 -> min_us, cmd=0 -> neutral, cmd=1 -> max_us
        if cmd >= 0.0:
            us = self.neutral_us + cmd * (self.max_us - self.neutral_us)
        else:
            us = self.neutral_us + cmd * (self.neutral_us - self.min_us)

        # deadband: snap very small commands to neutral
        if self.deadband_us > 0.0:
            if abs(us - self.neutral_us) <= self.deadband_us:
                us = self.neutral_us
        return us

    def us_to_angle(self, us: float) -> float:
        """
        Convert a microsecond pulse width in [min_us, max_us] to a 0..180
        rotation angle usable by the user's PCA9685.setRotationAngle.
        """
        # Avoid division by zero
        span = (self.max_us - self.min_us) if (self.max_us - self.min_us) != 0 else 1.0
        ratio = (us - self.min_us) / span
        angle = ratio * 180.0
        # Clip to typical angle bounds
        return max(0.0, min(180.0, angle))

    def apply_slew_limit(self, target_us: float, dt: float) -> float:
        if self.slew_us_per_s <= 0.0:
            return target_us
        max_delta = self.slew_us_per_s * dt
        delta = target_us - self.current_us
        if abs(delta) <= max_delta:
            return target_us
        return self.current_us + math.copysign(max_delta, delta)

    def update_output(self):
        now = self.get_clock().now()
        dt = (now - getattr(self, "_prev_time", now)).nanoseconds / 1e9
        self._prev_time = now

        # Watchdog: go neutral if stale
        if (now - self.last_msg_time) > self.watchdog_timeout:
            target_us = self.neutral_us
        else:
            target_us = self.map_cmd_to_us(self.current_cmd)

        # Slew limiting
        target_us = self.apply_slew_limit(target_us, dt if dt > 0 else 0.0)

        try:
            # Convert microseconds to rotation angle (0..180) and send via the
            # working PCA9685 API.
            angle = int(self.us_to_angle(target_us))
            try:
                self.pwm.setRotationAngle(self.channel, angle)
            except Exception:
                # fallback: some APIs may expose set_rotation_angle or set_angle
                try:
                    self.pwm.set_rotation_angle(self.channel, angle)
                except Exception:
                    # last resort: try set_pwm_us-like method if present
                    try:
                        self.pwm.set_pwm_us(self.channel, int(target_us), period_us=self.period_us)
                    except Exception as e:
                        raise
            self.current_us = target_us
        except Exception as e:
            self.get_logger().error(f'PWM write error: {e}')
    
    def destroy_node(self):
        # Fail-safe: neutral on shutdown
        try:
            # send neutral and release shared PCA holder
            try:
                # send neutral using rotation angle mapping
                try:
                    self.pwm.setRotationAngle(self.channel, int(self.us_to_angle(self.neutral_us)))
                except Exception:
                    try:
                        self.pwm.set_pwm_us(self.channel, int(self.neutral_us), period_us=self.period_us)
                    except Exception:
                        pass
            except Exception:
                pass
            # try to gracefully close the PCA9685 instance if available
            try:
                self.pwm.exit_PCA9685()
            except Exception:
                pass
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    # Support running multiple instances in one process via repeated
    # --node-name=NAME arguments. If none provided, single instance with
    # a unique anonymous name will be created.
    names = []
    count = None
    base_name = 'thruster'
    if args is None:
        argv = sys.argv[1:]
    else:
        argv = list(args)
    for a in argv:
        if a.startswith('--node-name='):
            names.append(a.split('=', 1)[1])
        elif a.startswith('--count='):
            try:
                count = int(a.split('=', 1)[1])
            except Exception:
                count = None
        elif a.startswith('--base-name='):
            base_name = a.split('=', 1)[1]

    # If explicit names weren't provided but a count was, generate numbered names
    if not names and count is not None and count > 0:
        names = [f"{base_name}_{i}" for i in range(count)]

    rclpy.init(args=args)
    try:
        if len(names) <= 1:
            node = ThrusterNode(node_name=(names[0] if names else None))
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.get_logger().info('Shutting down thruster node...')
                node.destroy_node()
        else:
            from rclpy.executors import MultiThreadedExecutor
            nodes = [ThrusterNode(node_name=n) for n in names]
            executor = MultiThreadedExecutor()
            for n in nodes:
                executor.add_node(n)
            try:
                executor.spin()
            except KeyboardInterrupt:
                pass
            finally:
                for n in nodes:
                    try:
                        n.get_logger().info('Shutting down thruster node...')
                        n.destroy_node()
                    except Exception:
                        pass
                executor.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()