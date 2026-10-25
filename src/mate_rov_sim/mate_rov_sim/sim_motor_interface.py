#!/usr/bin/env python3
"""
sim_motor_interface: the simulator's stand-in for the PCA9685, the ESCs and
the motors themselves.

On the real ROV, thruster_pkg/pwm_node turns duty cycles in [-1, 1] into
pulse widths and writes them to the PCA9685. In simulation we run the SAME
pwm_node with simulate:=true (no hardware writes), and this node listens to
what pwm_node would have written: its <motor topic>/response_pwm duty cycle,
which already has pwm_node's clamping, slew limiting and watchdog applied.

Every motor in thruster_pkg/config/individual_motor_config.yaml is handled:
  /thruster/<name>   ->  pulse width (us)  ->  thrust (N)
      -> /sim/<name>_joint/cmd               -> Gazebo Thruster plugin
  /arm/motor_<name>  ->  joint speed (rad/s; m/s for the claw)
      -> /sim/arm_<name>_joint/cmd           -> Gazebo JointController

Subscribes:
  <motor topic><input_suffix>   std_msgs/Float32  duty cycle in [-1, 1]
      input_suffix '/response_pwm' (default) = after pwm_node
                   ''                        = straight from controller /
                                               arm_controller (no pwm_node)
  /controls/precision_mode      std_msgs/Bool     switches the thrusters' PWM
                                                  range, exactly like pwm_node
Publishes:
  /sim/<joint>/cmd              std_msgs/Float64  (bridged to Gazebo)
"""
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from nuwave_utils_pkg.file_helpers import load_yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float64


def joint_name_for(topic):
    """Gazebo joint driven by a motor topic, or None if it isn't a thruster or arm motor."""
    base = topic.rstrip('/').rsplit('/', 1)[-1]
    if topic.startswith('/thruster/'):
        return base + '_joint'                       # /thruster/thruster_fll -> thruster_fll_joint
    if topic.startswith('/arm/'):
        return 'arm_' + base.removeprefix('motor_') + '_joint'  # /arm/motor_claw -> arm_claw_joint
    return None


class SimMotorInterface(Node):
    def __init__(self):
        super().__init__('sim_motor_interface')
        # '' = the same file pwm_node loads by default
        self.declare_parameter('motor_config', '')
        self.declare_parameter('input_suffix', '/response_pwm')
        self.declare_parameter('precision_mode_default', False)
        self.declare_parameter('output_topic_format', '/sim/{}/cmd')
        # Pulse width -> thrust lookup table (linear interpolation between points), for an
        # ESC whose neutral is esc_neutral_us. Defaults: T200 at 12 V with the ESC deadband.
        # Each motor's table is shifted by (its neutral_us - esc_neutral_us), so the thruster
        # stops exactly at the neutral_us in individual_motor_config.yaml.
        self.declare_parameter('pwm_points', [1100.0, 1475.0, 1525.0, 1900.0])
        self.declare_parameter('thrust_points', [-28.6, 0.0, 0.0, 36.4])
        self.declare_parameter('esc_neutral_us', 1500.0)
        self.declare_parameter('arm_max_speed', 0.8)     # rad/s at duty cycle +-1
        self.declare_parameter('claw_max_speed', 0.03)   # m/s at duty cycle +-1
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        gp = self.get_parameter
        config_path = gp('motor_config').value or os.path.join(
            get_package_share_directory('thruster_pkg'), 'config', 'individual_motor_config.yaml')
        self.precision_mode_enabled = bool(gp('precision_mode_default').value)
        self.pwm_pts = np.array(gp('pwm_points').value, dtype=float)
        self.thr_pts = np.array(gp('thrust_points').value, dtype=float)
        if len(self.pwm_pts) != len(self.thr_pts) or np.any(np.diff(self.pwm_pts) <= 0):
            raise RuntimeError('pwm_points must be increasing and the same length as thrust_points')
        self.esc_neutral = float(gp('esc_neutral_us').value)
        self.arm_speed = float(gp('arm_max_speed').value)
        self.claw_speed = float(gp('claw_max_speed').value)
        self.timeout = float(gp('cmd_timeout').value)
        suffix = gp('input_suffix').value
        fmt = gp('output_topic_format').value

        self.get_logger().info(f'Loading motor config from: {config_path}')
        self.motors = []
        for mot in load_yaml(config_path).get('motors', []):
            topic = mot.get('topic', '')
            joint = joint_name_for(topic)
            if joint is None:
                self.get_logger().warn(f'{topic}: not under /thruster/ or /arm/, ignoring')
                continue
            m = dict(mot, joint=joint, thruster=topic.startswith('/thruster/'),
                     duty=0.0, last_time=None)
            m['pub'] = self.create_publisher(Float64, fmt.format(joint), 10)
            self.create_subscription(Float32, topic + suffix,
                                     lambda msg, m=m: self.on_duty(msg, m), 10)
            self.motors.append(m)
        if not self.motors:
            raise RuntimeError(f'No motors found in {config_path}')
        self._apply_precision_mode()

        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, '/controls/precision_mode', self.on_precision_mode, qos_state)
        self.create_timer(1.0 / gp('publish_rate').value, self.on_timer)

        n_thr = sum(m['thruster'] for m in self.motors)
        self.get_logger().info(
            f'Driving {n_thr} thrusters and {len(self.motors) - n_thr} arm joints '
            f'from <motor topic>{suffix or " (raw duty cycles)"}')

    # Same range selection as pwm_node._apply_precision_mode_to_motor
    def _apply_precision_mode(self):
        for m in self.motors:
            keys = ('speed_min_us', 'speed_max_us', 'precision_min_us', 'precision_max_us')
            if any(m.get(k) is None for k in keys):
                continue
            mode = 'precision' if self.precision_mode_enabled else 'speed'
            m['min_us'], m['max_us'] = m[f'{mode}_min_us'], m[f'{mode}_max_us']

    def on_precision_mode(self, msg):
        if bool(msg.data) != self.precision_mode_enabled:
            self.precision_mode_enabled = bool(msg.data)
            self._apply_precision_mode()
            self.get_logger().info(f'Precision mode: {"ON" if self.precision_mode_enabled else "OFF"}')

    def on_duty(self, msg, m):
        duty = float(msg.data)
        if not np.isfinite(duty):
            return
        m['duty'] = float(np.clip(duty, -1.0, 1.0))
        m['last_time'] = self.get_clock().now()

    def duty_to_us(self, duty, m):
        """Same mapping as pwm_node._map_dutycycle_to_pwm."""
        neutral = m['neutral_us']
        if duty >= 0.0:
            return neutral + duty * (m['max_us'] - neutral)
        return neutral + duty * (neutral - m['min_us'])

    def us_to_thrust(self, us, m):
        return float(np.interp(us - m['neutral_us'] + self.esc_neutral, self.pwm_pts, self.thr_pts))

    def on_timer(self):
        now = self.get_clock().now()
        for m in self.motors:
            fresh = (m['last_time'] is not None and
                     (now - m['last_time']).nanoseconds * 1e-9 < self.timeout)
            duty = m['duty'] if fresh else 0.0
            if m['thruster']:
                value = self.us_to_thrust(self.duty_to_us(duty, m), m)
            else:
                value = duty * (self.claw_speed if 'claw' in m['joint'] else self.arm_speed)
            m['pub'].publish(Float64(data=value))


def main():
    rclpy.init()
    node = SimMotorInterface()
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
