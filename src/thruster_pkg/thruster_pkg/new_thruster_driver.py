import time
import rclpy
import Jetson.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.duration import Duration

from .PCA9685 import PCA9685

# ---- CONFIG ----
LOW_ANGLE = 0
HIGH_ANGLE = 180
STOP_ANGLE = 90

class ThrusterNode(Node):
    """
    Subscribes: std_msgs/Float32 representing PWM signals 
    Safety: clamp, watchdog timeout -> neutral, optional slew limiting
    """

    def __init__(self):
        super().__init__('thruster_node')
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

        self.deadband_us = self.get_parameter('deadband_us').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.watchdog_timeout = Duration(
                seconds=self.get_parameter('watchdog_timeout_s').value
                )
        self.slew_us_per_s = self.get_parameter('slew_us_per_s').value
        # Derived Vals
        self.period_us = 1_000_000.0 / self.pwm_freq

        if not topic: 
            topic = f"thruster_{self.channel}"

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

        self._arm()

        # Ros Wiring
        self.sub = self.create_subscription(Float32, topic, self.cmd_callback, 10)
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.update)

        self.get_logger().info(
                f'Listening on "{topic}" for Float32 (us), '
                f'range [{self.min_us, self.max_us}], neutral_us={self.neutral_us}'
                )

    def _arm(self):
        """Simple arming sequence and set to neutral/stop."""
        print("Arming thruster on channel", self.channel)
        try:
            self.pwm.setRotationAngle(self.channel, STOP_ANGLE)
        except Exception:
            pass
        print(f" -> {STOP_ANGLE} (stop / neutral)")
        time.sleep(2.0)
        
    def _write_us(self, us : float):
        """
        Clamp, convert and write to hardware.
        """
        us = max(self.min_us, min(self.max_us, us))
        us_centered = us - self.neutral_us
        angle = 0.0
        if us_centered >= 0.0:
            angle = us_centered / (self.max_us - self.neutral_us)
        else:
            angle = us_centered / (self.neutral_us - self.min_us)
        self._write_angle(angle)
    
    def _write_angle(self, val : float):
        # Map normalized command [-1,1] to angle [LOW_ANGLE..HIGH_ANGLE]
        if val > 1.0:
            # assume user might send direct angle if >1
            angle = max(LOW_ANGLE, min(HIGH_ANGLE, int(val)))
        elif val < -1.0:
            # if very negative, ignore and clamp
            angle = LOW_ANGLE
        else:
            if val >= 0.0:
                angle = STOP_ANGLE + val * (HIGH_ANGLE - STOP_ANGLE)
            else:
                angle = STOP_ANGLE + val * (STOP_ANGLE - LOW_ANGLE)

        angle = int(max(LOW_ANGLE, min(HIGH_ANGLE, angle)))
        self.get_logger().debug(f'Angle: {angle}')

        try:
            self.pwm.setRotationAngle(self.channel, angle)
        except Exception as e:
            self.get_logger().error(f'Failed to send angle: {e}')

    # Callbacks
    def cmd_callback(self, msg : Float32):
        """
        Signal to SPIN BABY SPIN
        """
        cmd = float(msg.data)
        if cmd > HIGH_ANGLE:
            self._write_us(cmd)
        else:
            self._write_angle(cmd)
        self.target_us = max(self.min_us, min(self.max_us, msg.data))
        self.last_msg_time = self.get_clock().now()

    # TODO: reintegrate updates independent of callback if possible to allow different publishing and update rates
    def update(self):
        pass
    #     now = self.get_clock().now()
    #     dt = (now - getattr(self, "_prev_time", now)).nanoseconds / 1e9
    #     self._prev_time = now

    #     # Watchdog: go neutral if stale
    #     if (now - self.last_msg_time) > self.watchdog_timeout:
    #         target_us = self.neutral_us
    #     else:
    #         target_us = self.map_cmd_to_us(self.current_cmd)

    #     # Skew Limiting
    #     if self.slew_us_per_s > 0.0 and dt > 0.0:
    #         max_delta = self.slew_us_per_s * dt
    #         delta = target_us - self.current_us
    #         if abs(delta) > max_delta:
    #             target = self.current_us + math.copysign(max_delta, delta)
    #     # Write 
    #     if self.pwm is not None:
    #         try:
    #             self._write_us(target)
    #         except Exception as e:
    #             self.get_logger().error(f'PWM write error: {e}')

    def destroy_node(self):
        # send neutral and cleanup
        try:
            self.pwm.setRotationAngle(self.channel, STOP_ANGLE)
        except Exception as e:
            self.get_logger().error(f'Failed to send stop angle on shutdown: {e}')
        try:
            self.pwm.exit_PCA9685()
        except Exception:
            pass

        if GPIO is not None:
            try:
                GPIO.cleanup()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init()
    node = ThrusterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt — shutting down thruster node.")
    finally:
        node.get_logger().info('Shutting down thruster node...')
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
