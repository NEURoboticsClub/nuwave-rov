import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.duration import Duration
from .pwm_scribe_base import PWMScribeBase
from .pwm_scribe_arduino import PWMScribeArduino
from .pwm_scribe_hat import PWMScribeHat
from nuwave_utils_pkg.file_helpers import load_yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

class PWMNode(Node):
    """
    Subscribes: std_msgs/Float32 representing PWM signals 
    Safety: clamp, watchdog timeout -> neutral, optional slew limiting
    """

    def __init__(self, node_name='pwm_node', **kwargs):
        super().__init__(node_name, **kwargs)
        # Params
        self.declare_parameter('pwm_freq_hz', 50)             # ESC expects ~50 Hz
        self.declare_parameter('update_rate_hz', 50.0)        # write rate to the hat
        self.declare_parameter('watchdog_timeout_s', 0.5)     # neutral if no msg in this time
        self.declare_parameter('slew_us_per_s', 750.0)        # 0 to disable rate limit; else max change per sec
        self.declare_parameter('simulate', False)
        self.declare_parameter('pwm_hardware', 'hat')     # 'arduino' or 'hat'
        pkg_share = get_package_share_directory('thruster_pkg')
        self.declare_parameter(
                'individual_motor_config', 
                os.path.join(pkg_share, 'config', 'individual_motor_config.yaml')
                )
        # self.declare_parameter(
        #         'general_motor_config', 
        #         os.path.join(pkg_share, 'config', 'general_motor_config.yaml')
        #         )
        # hardware-specific
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x40)
        port = self.declare_parameter("firmata_port", "/dev/ttyACM0")
        pins = self.declare_parameter("firmata_pins", [9,10,11,12])
        # Read Params
        bus = self.get_parameter('i2c_bus').value
        addr = self.get_parameter('i2c_address').value
        self.pwm_freq = self.get_parameter('pwm_freq_hz').value

        pwm_hardware = self.get_parameter('pwm_hardware').get_parameter_value().string_value

        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.watchdog_timeout = Duration(
                seconds=self.get_parameter('watchdog_timeout_s').value
                )
        self.simulate = self.get_parameter('simulate').value
        # Derived Vals
        self.period_us = 1_000_000.0 / self.pwm_freq

        # PCA stuff
        # Adding a simulation flag, so we can see the driver code work without the PCA9865 needing to be connected

        if self.simulate:
            self.pwm = None
            self.get_logger().warn('Sim mode: no hardware writes')
        else:
            try:
                if pwm_hardware == "hat":
                    addr = self.get_parameter("i2c_address").value
                    bus = self.get_parameter("i2c_bus").value
                    self.pwm = PWMScribeHat(bus, addr)
                elif pwm_hardware == "arduino":
                    port = self.get_parameter("firmata_port").get_parameter_value().string_value # this may be wrong
                    pins = self.get_parameter("firmata_pins").value # this is almost definitely wrong
                    self.pwm = PWMScribeArduino(port, pins)
                self.pwm.setup(int(self.pwm_freq))
                self.get_logger().info("Hardware initialized")
            except Exception as e:
                self.pwm = None
                self.get_logger().error(f'PCA9685 init failed: {e}') # we should add some retry logic here
        
        individual_motor_config_path = self.get_parameter('individual_motor_config').value

        self.get_logger().info(f"Loading individual thruster config from: {individual_motor_config_path}")

        # Configure Thrusters and Allocation
        individual_config = load_yaml(individual_motor_config_path)

        self.motors = individual_config.get('motors', [])
        self.motors_lookup = {mot['topic']: mot for mot in self.motors}

        if not self.motors:
            raise ValueError(f"No motors found in config")

        # Validate PWM config ranges up front: min < neutral < max is required, otherwise
        # the dutycycle <-> pwm mapping divides by zero at runtime
        for mot in self.motors:
            min_us = mot.get('min_us')
            neutral_us = mot.get('neutral_us', 1500)
            max_us = mot.get('max_us')
            if min_us is None or max_us is None or not (min_us < neutral_us < max_us):
                raise ValueError(
                    f"Motor {mot.get('topic')} has invalid PWM range: "
                    f"min_us={min_us}, neutral_us={neutral_us}, max_us={max_us} "
                    f"(need min_us < neutral_us < max_us)"
                )

        self._arm()

        # Ros Wiring
        self.motor_subs = []
        self.response_pubs = []
        for mot in self.motors:
            sub = self.create_subscription(Float32, mot.get('topic'), lambda msg, m=mot: self.cmd_callback(msg, m.get('topic')), 10)
            self.motor_subs.append(sub)
            mot['target_us'] = mot.get('neutral_us', 1500)
            mot['current_us'] = mot.get('neutral_us', 1500)
            pub = self.create_publisher(Float32, mot.get('topic') + "/response_pwm", 10)
            self.response_pubs.append(pub)
            self.get_logger().info(
                f"Listening on {mot.get('topic')} for Float32 (us), "
                f"range [{mot.get('min_us'), mot.get('max_us')}], neutral_us={mot.get('neutral_us', 1500)}, "
                f"slew_rate={mot.get('slew_us_per_s')}"
                )
            now = self.get_clock().now()
            mot['last_msg_time'] = now
            mot['prev_time'] = now
            mot['got_first_cmd'] = False
            mot['watchdog_tripped'] = False
        
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.update)

    
    # takes dutycycle (-1 to 1) and maps that to PWM range for each motor
    def _map_dutycycle_to_pwm(self, dutycycle : float, mot : dict) -> float:
        # These are nx1 vectors for each thrusters configured pwm ranges
        min_us = mot['min_us']
        neutral_us = mot['neutral_us']
        max_us = mot['max_us']
        
        # This normalizes the forces to be within -1 and 1
        normalized = np.clip(dutycycle, -1.0, 1.0)
        
        # If normalized >= 0, which is forward we linear interp the pwm to be some point between neutral and max
        # Same is applied for reverse but with neutral_us being the leading term.
        pwm = neutral_us + normalized * (max_us - neutral_us) if normalized >= 0 else neutral_us + normalized * (neutral_us - min_us)
        return pwm
    
    def _map_pwm_to_dutycycle(self, pwm : float, mot : dict) -> float:
        # These are nx1 vectors for each thrusters configured pwm ranges
        min_us = mot['min_us']
        neutral_us = mot['neutral_us']
        max_us = mot['max_us']
        
        # This normalizes the forces to be within -1 and 1
        normalized = pwm - neutral_us
        
        # If normalized >= 0, which is forward we linear interp the pwm to be some point between neutral and max
        # Same is applied for reverse but with neutral_us being the leading term.
        dutycycle = normalized / (max_us - neutral_us) if normalized >= 0 else normalized / (neutral_us - min_us)
        return dutycycle

    def _arm(self):
        """Simple arming sequence and set to neutral/stop."""
        if self.pwm is not None:
            for mot in self.motors:
                print("Arming thruster on channel", mot['channel'])
                try:
                    self.pwm.set_pwm(mot['channel'], mot['neutral_us'])
                except Exception:
                    pass
            print(f" -> {mot['neutral_us']} (stop / neutral)")
        time.sleep(2.0)
    
    def _write_all_us(self, us: float):
        for mot in self.motors:
            self.pwm.set_pwm(mot['channel'], us)

    # Callbacks
    def cmd_callback(self, msg : Float32, topic : str):
        """
        Signal to SPIN BABY SPIN
        """
        mot = self.motors_lookup.get(topic)
        if mot is None:
            self.get_logger().warn(f"Received command for unknown motor topic: {topic}")
            return

        cmd = float(msg.data)
        if not np.isfinite(cmd):
            self.get_logger().warn(f"Ignoring non-finite command on {topic}: {cmd}")
            return

        mot['target_us'] = max(mot['min_us'], min(mot['max_us'], self._map_dutycycle_to_pwm(cmd, mot)))
        mot['last_msg_time'] = self.get_clock().now()
        mot['got_first_cmd'] = True

    # TODO: reintegrate updates independent of callback if possible to allow different publishing and update rates
    def update(self):
        now = self.get_clock().now()
        dt = [(now - mot['prev_time']).nanoseconds / 1e9 for mot in self.motors]
        for mot in self.motors:
            mot['prev_time'] = now
        
        for i, mot in enumerate(self.motors):
            try:
                # Watchdog: ramp to neutral if commands stop arriving; log once per timeout event
                timed_out = mot['got_first_cmd'] and (now - mot['last_msg_time']) > self.watchdog_timeout
                if timed_out:
                    if not mot.get('watchdog_tripped', False):
                        self.get_logger().warn(
                            f"Watchdog timeout on {mot.get('topic')}: no command received, ramping to neutral."
                        )
                        mot['watchdog_tripped'] = True
                    mot['target_us'] = mot['neutral_us']
                else:
                    mot['watchdog_tripped'] = False

                # Slew limiting
                target = mot['target_us']
                slew_us_per_s = mot.get('slew_us_per_s', 0.0)
                if slew_us_per_s > 0.0 and dt[i] > 0.0:
                    max_delta = slew_us_per_s * dt[i]
                    delta = target - mot['current_us']
                    if abs(delta) > max_delta:
                        target = mot['current_us'] + (max_delta if delta > 0 else -max_delta)

                mot['current_us'] = target

                if self.pwm is not None:
                    try:
                        self.pwm.set_pwm(mot['channel'], target)
                    except Exception as e:
                        self.get_logger().error(f"PWM write error on channel {mot['channel']}: {e}")

                response_dutycycle_message = Float32()
                response_dutycycle_message.data = float(self._map_pwm_to_dutycycle(target, mot))
                self.response_pubs[i].publish(response_dutycycle_message)
            except Exception as e:
                self.get_logger().error(f"Error updating motor {mot.get('topic')}: {e}")

    def destroy_node(self):
        # send neutral and cleanup
        if self.pwm is not None:
            for mot in self.motors:
                try:
                    self.pwm.set_pwm(mot['channel'], mot['neutral_us'])
                except Exception as e:
                    self.get_logger().error(f"Failed to send stop angle to channel {mot['channel']} on shutdown: {e}")
        try:
            self.pwm.shutdown()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init()
    node = PWMNode()

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
