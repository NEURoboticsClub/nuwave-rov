import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.duration import Duration
import smbus  

class PCA9685:
    _MODE1 = 0x00
    _MODE2 = 0x01
    _PRESCALE = 0xFE
    _LED0_ON_L = 0x06

    def __init__(self, bus_num=7, address=0x40, freq_hz=50):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        # Reset MODE1
        self.bus.write_byte_data(self.address, self._MODE1, 0x00)
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)

    def set_pwm_freq(self, freq_hz):
        osc_clock = 25_000_000.0  # 25 MHz
        prescaleval = osc_clock / (4096.0 * float(freq_hz)) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.bus.read_byte_data(self.address, self._MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.bus.write_byte_data(self.address, self._MODE1, newmode)
        self.bus.write_byte_data(self.address, self._PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self._MODE1, oldmode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self._MODE1, oldmode | 0x80)  # restart

    def set_pwm_us(self, channel: int, pulse_us: float, period_us: int = 20000):
        # Bound to 1250-1750 us for safety
        pulse_us = max(1250, min(1750, int(pulse_us)))
        ticks = int(pulse_us * 4096.0 / period_us)  # 0..4095
        on = 0
        off = ticks
        reg = self._LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(
            self.address,
            reg,
            [on & 0xFF, on >> 8, off & 0xFF, off >> 8]
        )

class TD7ThrusterNode(Node):
    """
    Subscribes: std_msgs/Float32 on <topic> (default: /cmd_thrust), range [-1, 1].
    Maps to PWM us around neutral and writes to PCA9685 channel.
    Safety: clamp, watchdog timeout -> neutral, optional slew limiting.
    """

    def __init__(self):
        super().__init__('td7_thruster_node')

        self.declare_parameter('topic', 'cmd_thrust')
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

        # Init hardware
        self.pca = PCA9685(bus_num=bus, address=addr, freq_hz=int(pwm_freq))
        self.get_logger().info(f'PCA9685 on bus {bus}, addr 0x{addr:02X}, freq {pwm_freq} Hz')

        # ROS wiring
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

        # optional deadband: snap very small commands to neutral
        if self.deadband_us > 0.0:
            if abs(us - self.neutral_us) <= self.deadband_us:
                us = self.neutral_us
        return us

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
            self.pca.set_pwm_us(self.channel, target_us, period_us=self.period_us)
            self.current_us = target_us
        except Exception as e:
            self.get_logger().error(f'PWM write error: {e}')

    def destroy_node(self):
        # Fail-safe: neutral on shutdown
        try:
            self.pca.set_pwm_us(self.channel, self.neutral_us, period_us=self.period_us)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TD7ThrusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down thruster node...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()