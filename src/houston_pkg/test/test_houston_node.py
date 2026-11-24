"""Test production node behavior with deterministic ROS and hardware boundaries."""

import copy
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import pytest


@pytest.fixture
def runtime(monkeypatch):
    """Keep imports, ROS time, publishers, and node parameters local to each test."""
    class Time:
        def __init__(self, seconds=0):
            self.nanoseconds = round(seconds * 1e9)

        def __sub__(self, other):
            return Time((self.nanoseconds - other.nanoseconds) / 1e9)

        def __gt__(self, other):
            return self.nanoseconds > other.nanoseconds

        def to_msg(self):
            return self.nanoseconds

    class Node:
        overrides = {}
        instances = []

        def __init__(self, name, **kwargs):
            self.name = name
            self.parameters = self.overrides.copy()
            self.logger = Mock()
            self.now = 0.0
            self.publishers = []
            self.subscriptions = []
            self.timers = []
            self.destroyed = False
            self.instances.append(self)

        def declare_parameter(self, name, default):
            self.parameters.setdefault(name, default)
            return self.get_parameter(name)

        def get_parameter(self, name):
            value = self.parameters[name]
            return NS(value=value, get_parameter_value=lambda: NS(
                integer_value=value, double_value=value, string_value=value,
                bool_value=value, double_array_value=value))

        def get_logger(self):
            return self.logger

        def get_clock(self):
            return NS(now=lambda: Time(self.now))

        def create_publisher(self, message_type, topic, qos):
            pub = Mock(topic=topic, qos=qos, message_type=message_type, messages=[])
            pub.publish.side_effect = lambda msg: pub.messages.append(copy.deepcopy(msg))
            self.publishers.append(pub)
            return pub

        def create_subscription(self, message_type, topic, callback, qos):
            sub = NS(topic=topic, callback=callback, qos=qos, message_type=message_type)
            self.subscriptions.append(sub)
            return sub

        def create_timer(self, period, callback):
            timer = NS(period=period, callback=callback)
            self.timers.append(timer)
            return timer

        def destroy_node(self):
            self.destroyed = True

    def install(name, **attributes):
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    def vector(x=0.0, y=0.0, z=0.0):
        return NS(x=x, y=y, z=z)

    def quaternion(x=0.0, y=0.0, z=0.0, w=0.0):
        return NS(x=x, y=y, z=z, w=w)

    def twist():
        return NS(linear=vector(), angular=vector())

    def imu():
        return NS(header=NS(stamp=None, frame_id=''), orientation=quaternion(),
                  angular_velocity=vector(), linear_acceleration=vector())

    def image():
        return NS(header=NS(stamp=None, frame_id=''), format='', data=b'')

    ros = install('rclpy', init=Mock(), spin=Mock(), shutdown=Mock(), ok=Mock(return_value=True))
    install('rclpy.node', Node=Node)
    install('rclpy.duration', Duration=Time)
    policies = NS(BEST_EFFORT=1, RELIABLE=2, KEEP_LAST=1, TRANSIENT_LOCAL=1, VOLATILE=2)
    install('rclpy.qos', QoSProfile=NS, ReliabilityPolicy=policies,
            HistoryPolicy=policies, DurabilityPolicy=policies)
    install('geometry_msgs')
    install('geometry_msgs.msg', Twist=twist, Quaternion=quaternion, Vector3=vector)
    install('std_msgs')
    install('std_msgs.msg', Float32=NS, Float32MultiArray=NS, Bool=NS, Empty=NS, String=NS)
    install('sensor_msgs')
    install('sensor_msgs.msg', Imu=imu, CompressedImage=image, Image=image, Joy=NS)
    install('ament_index_python')
    install('ament_index_python.packages', get_package_share_directory=lambda package:
            str(Path(__file__).resolve().parents[3] / 'src' / package))
    install('nuwave_utils_pkg')
    install('nuwave_utils_pkg.file_helpers', load_yaml=lambda path:
            __import__('yaml').safe_load(Path(path).read_text()))

    def load(package, filename):
        alias = '_test_' + filename.removesuffix('.py')
        install(alias, __path__=[str(Path(__file__).parents[1] / package)])
        fullname = alias + '.' + filename.removesuffix('.py')
        spec = importlib.util.spec_from_file_location(
            fullname, Path(__file__).parents[1] / package / filename)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, fullname, module)
        spec.loader.exec_module(module)
        return module

    return NS(Node=Node, Time=Time, install=install, load=load, ros=ros,
              vector=vector, quaternion=quaternion, twist=twist, imu=imu, image=image)


@pytest.fixture
def houston(runtime):
    return runtime.load('houston_pkg', 'houston_node.py')


def test_constructor_uses_real_config_and_publishes_latched_initial_modes(houston, runtime):
    runtime.Node.overrides = {'publish_rate_hz': 20, 'expo_enabled_default': True,
                              'precision_mode_default': True, 'joy_thruster': '/test/drive',
                              'joy_arm': '/test/arm'}
    node = houston.Houston()
    assert node.name == 'houston'
    assert set(node.joy_map) == {'thruster_control', 'arm_control'}
    assert node.thruster_joy_sub.topic == '/test/drive'
    assert node.arm_joy_sub.topic == '/test/arm'
    assert node.stabilizer_sub.topic == '/stabilizer/commands'
    assert node.timers[0].period == pytest.approx(0.05)
    assert node.timers[0].callback == node._publish_loop
    assert [pub.messages[-1].data for pub in [node.expo_mode_pub, node.precision_mode_pub,
                                             node.stabilize_mode_pub]] == [True, True, False]
    assert node.expo_mode_pub.qos.depth == 1
    assert node.expo_mode_pub.qos.durability == 1
    assert not node.twist_pub.messages and not node.arm_pub.messages


@pytest.mark.parametrize('state, callback, publisher', [
    ('expo_enabled', 'gui_expo_toggle_callback', 'expo_mode_pub'),
    ('precision_mode_enabled', 'gui_precision_mode_toggle_callback', 'precision_mode_pub'),
    ('stabilize_enabled', 'gui_stabilize_toggle_callback', 'stabilize_mode_pub')])
def test_gui_mode_callbacks_publish_only_on_change(houston, state, callback, publisher):
    node = houston.Houston()
    change = getattr(node, callback)
    pub = getattr(node, publisher)
    change(NS(data=True))
    change(NS(data=True))
    assert getattr(node, state) is True and len(pub.messages) == 2
    if state == 'stabilize_enabled':
        assert len(node.capture_pub.messages) == 1
    change(NS(data=False))
    assert getattr(node, state) is False and len(pub.messages) == 3


@pytest.mark.parametrize('button, state', [('expo_toggle', 'expo_enabled'),
    ('precision_toggle', 'precision_mode_enabled'), ('stabilize_toggle', 'stabilize_enabled')])
def test_controller_toggles_only_on_rising_button_edges(houston, button, state):
    node = houston.Houston()
    for pressed, expected in [(1, True), (1, True), (0, True), (1, False)]:
        node.publish_twist({}, {button: pressed})
        assert getattr(node, state) is expected


def test_exponential_curve_has_deadzone_endpoints_symmetry_and_monotonicity(houston):
    node = houston.Houston()
    assert [node.scale_controller_input(x) for x in [-0.05, 0, 0.05]] == [0, 0, 0]
    assert node.scale_controller_input(1) == pytest.approx(1)
    assert node.scale_controller_input(-1) == pytest.approx(-1)
    assert node.scale_controller_input(-0.5) == pytest.approx(-node.scale_controller_input(0.5))
    values = [node.scale_controller_input(x) for x in [0.1, 0.25, 0.5, 0.75, 1]]
    assert values == sorted(values)
    assert 0 < values[2] < 0.5


@pytest.mark.parametrize('raw, extra, expected', [
    (0.069, {}, 0), (0.07, {}, 0), (0.535, {}, 0.5), (-0.535, {}, -0.5),
    (0.535, {'invert': True}, -0.5), (0.535, {'sensitivity': 3}, 1),
    (1.0, {'deadzone': 2.0}, 1), (0.5, {'deadzone': -1}, 0.5),
    (0.5, {'deadzone': 0, 'scale': 'logarithmic'}, 0.4054651081081644),
    (0.5, {'deadzone': 0, 'scale': 'exponential'}, 0.5)])
def test_axis_parsing_deadzone_inversion_sensitivity_and_scaling(houston, raw, extra, expected):
    node = houston.Houston()
    node.joy_map = {'test': [{'axis': 'test_axis', 'input': 0, **extra}]}
    parsed = node.parse_joystick(NS(axes=[raw], buttons=[]), 'test')
    assert parsed == {'axis': {'test_axis': pytest.approx(expected)}, 'button': {}}
    node.expo_enabled = True
    if extra.get('scale') == 'exponential':
        assert node.parse_joystick(NS(axes=[raw], buttons=[]), 'test')['axis']['test_axis'] < expected


@pytest.mark.parametrize('index', [-1, 99])
def test_out_of_range_axis_and_button_indexes_do_not_read_other_controls(houston, index):
    node = houston.Houston()
    node.joy_map = {'test': [{'axis': 'missing', 'input': index},
                             {'button': 'missing', 'input': index}]}
    assert node.parse_joystick(NS(axes=[0.8], buttons=[1]), 'test') == {
        'axis': {'missing': 0.0}, 'button': {'missing': 0}}


def test_button_inversion_and_malformed_section_fallback(houston):
    node = houston.Houston()
    node.joy_map = {'test': [{'button': 'inverted', 'input': 0, 'invert': True}],
                    'bad': {}, 'ignored': [{}]}
    assert node.parse_joystick(NS(axes=[], buttons=[0]), 'test')['button'] == {'inverted': 1}
    for section in ['bad', 'missing', 'ignored']:
        assert node.parse_joystick(NS(axes=[], buttons=[]), section) == {'axis': {}, 'button': {}}


@pytest.mark.parametrize('age, expected', [(0.499999999, 0.5), (0.5, 0.5), (0.500000001, 0)])
def test_thruster_watchdog_boundary(houston, age, expected):
    node = houston.Houston()
    node._publish_loop()
    assert not node.twist_pub.messages
    node.joy_map = {'thruster_control': [{'axis': 'drive_forward', 'input': 0}]}
    command = NS(axes=[0.535], buttons=[])
    node.joy_thruster_callback(command)
    assert node.last_joy_thruster_msg is command
    node.now = age
    node._publish_loop()
    assert node.twist_pub.messages[-1].linear.y == pytest.approx(expected)


def test_thruster_timeout_warns_once_and_recovers(houston):
    node = houston.Houston()
    command = NS(axes=[0] * 6, buttons=[0] * 6)
    node.joy_thruster_callback(command)
    node.now = 1.0
    node._publish_loop()
    node._publish_loop()
    assert node.joy_thruster_stale
    node.logger.warn.assert_called_once()
    node.joy_thruster_callback(command)
    node._publish_loop()
    assert not node.joy_thruster_stale


def test_twist_axis_mapping_and_six_element_arm_order(houston):
    node = houston.Houston()
    node.publish_twist({'strafe': 0.1, 'drive_forward': 0.2, 'up': 1, 'down': -1,
                        'pitch': 0.3, 'yaw': 0.4}, {'roll_right': 1, 'roll_left': 0})
    msg = node.twist_pub.messages[-1]
    assert [msg.linear.x, msg.linear.y, msg.linear.z] == [0.1, 0.2, 1]
    assert [msg.angular.x, msg.angular.y, msg.angular.z] == [0.3, 1, 0.4]
    node.publish_arm_commands({'base_yaw_input': 0.1, 'base_pitch_input': 0.2,
        'elbow_pitch_input': 0.3, 'wrist_yaw_input': 0.4, 'wrist_up_input': 0.8,
        'wrist_down_input': -0.2}, {'claw_open_input': 1})
    assert node.arm_pub.messages[-1].data == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 1])
    node.publish_arm_commands({}, {})
    assert node.arm_pub.messages[-1].data == [0] * 6


@pytest.mark.parametrize('age, enabled', [(0.499999999, True), (0.5, True), (0.500000001, False)])
def test_stabilizer_timeout_and_mixing_preserve_vector_ratios(houston, runtime, age, enabled):
    node = houston.Houston()
    node.set_stabilize_enabled(True, 'test')
    node.publish_twist({'strafe': 0.75}, {})
    assert node.twist_pub.messages[-1].linear.x == 0.75
    correction = runtime.twist()
    correction.linear.x, correction.linear.y = 0.75, 0.25
    correction.angular.x, correction.angular.y = 1, 0.5
    node.stabilizer_callback(correction)
    node.now = age
    node.publish_twist({'strafe': 0.75, 'drive_forward': 0.5, 'up': 1, 'down': -1,
                        'pitch': 1}, {})
    result = node.twist_pub.messages[-1]
    assert node.stabilize_enabled is enabled
    expected_linear = [1, 0.5, 2 / 3] if enabled else [0.75, 0.5, 1]
    expected_angular = [1, 0.25, 0] if enabled else [1, 0, 0]
    assert [result.linear.x, result.linear.y, result.linear.z] == pytest.approx(expected_linear)
    assert [result.angular.x, result.angular.y, result.angular.z] == pytest.approx(expected_angular)


def test_arm_callback_and_parse_errors_are_contained(houston, monkeypatch):
    node = houston.Houston()
    node.joy_map = {'arm_control': [{'axis': 'base_yaw_input', 'input': 0}]}
    message = NS(axes=[0.535], buttons=[])
    node.joy_arm_callback(message)
    assert node.last_joy_arm_msg is message
    assert node.arm_pub.messages[-1].data == pytest.approx([0.5, 0, 0, 0, 0, 0])
    monkeypatch.setattr(node, 'parse_joystick', Mock(side_effect=ValueError('bad config')))
    node.joy_arm_callback(message)
    node.joy_thruster_callback(message)
    node._publish_loop()
    assert node.logger.error.call_count == 2


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_cleanup(houston, runtime, error):
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            houston.main()
    else:
        houston.main()
    assert runtime.Node.instances[-1].destroyed
    runtime.ros.shutdown.assert_called_once_with()
