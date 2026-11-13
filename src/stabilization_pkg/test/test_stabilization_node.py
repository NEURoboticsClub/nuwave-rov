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
def stabilization(runtime):
    return runtime.load('stabilization_pkg', 'stabilization_node.py')


def test_constructor_loads_real_config_and_wires_callbacks(stabilization):
    node = stabilization.StabilizationNode()
    assert node.name == 'stabilization'
    assert node.imu_stabilization_params['p'] == 0.5
    assert node.imu_sub.topic == '/imu'
    assert node.imu_sub.callback == node.imu_callback
    assert node.capture_sub.topic == '/stabilizer/capture'
    assert node.capture_sub.callback == node.capture_callback
    assert node.sta_pub.topic == '/stabilizer/commands'
    assert node.desired_orientation.w == 1.0
    assert node.last_imu_orientation is None


@pytest.mark.parametrize('config', [None, [], 42, {'imu_stabilization': None}])
def test_malformed_or_empty_configuration_falls_back(stabilization, monkeypatch, config):
    monkeypatch.setattr(stabilization, 'load_yaml', lambda path: config)
    node = stabilization.StabilizationNode()
    assert node.imu_stabilization_params == {}
    assert node.depth_stabilization_params == {}


@pytest.mark.parametrize('left, right, expected', [
    ([1, 0, 0, 0], [0, 1, 0, 0], [0, 1, 0, 0]),
    ([0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]),
    ([0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, -1]),
])
def test_quaternion_product(stabilization, left, right, expected):
    assert stabilization.StabilizationNode._quat_mul(left, right) == pytest.approx(expected)


def test_setpoint_is_normalized_without_mutating_input(stabilization, runtime):
    node = stabilization.StabilizationNode()
    value = runtime.quaternion(x=3.0, w=4.0)
    assert node.set_desired_orientation(value)
    assert node.desired_orientation.x == pytest.approx(0.6)
    assert node.desired_orientation.w == pytest.approx(0.8)
    assert (value.x, value.w) == (3.0, 4.0)


@pytest.mark.parametrize('value', [0.0, 1e-8, float('nan'), float('inf')])
def test_invalid_setpoint_preserves_previous_orientation(stabilization, runtime, value):
    node = stabilization.StabilizationNode()
    before = node.desired_orientation
    assert not node.set_desired_orientation(runtime.quaternion(w=value))
    assert node.desired_orientation is before
    node.logger.warn.assert_called_once()


def test_identity_orientation_uses_gyro_damping_and_suppresses_yaw(stabilization, runtime):
    node = stabilization.StabilizationNode()
    node.imu_stabilization_params = {'p': 2.0, 'd': 0.5, 'max_stabilization_influence': 10}
    msg = runtime.imu()
    msg.orientation.w = 1.0
    msg.angular_velocity = runtime.vector(2, -4, 3)
    result = node._imu_to_twist_control(msg)
    assert [result.angular.x, result.angular.y, result.angular.z] == [-1, 2, 0]
    assert [result.linear.x, result.linear.y, result.linear.z] == [0, 0, 0]


def test_orientation_error_uses_shortest_arc_and_limits_influence(stabilization, runtime):
    node = stabilization.StabilizationNode()
    node.imu_stabilization_params = {'p': 3.0, 'max_stabilization_influence': 0.25}
    msg = runtime.imu()
    msg.orientation = runtime.quaternion(x=2 ** -0.5, w=2 ** -0.5)
    first = node._imu_to_twist_control(msg)
    msg.orientation = runtime.quaternion(x=-2 ** -0.5, w=-2 ** -0.5)
    second = node._imu_to_twist_control(msg)
    assert first.angular.x == pytest.approx(-0.25)
    assert second.angular.x == pytest.approx(first.angular.x)
    assert first.angular.y == first.angular.z == 0.0


@pytest.mark.parametrize('field, value', [('orientation', 0.0), ('orientation', float('nan')),
                                         ('orientation', float('inf')), ('gyro', float('nan'))])
def test_bad_imu_data_produces_neutral_command(stabilization, runtime, field, value):
    node = stabilization.StabilizationNode()
    msg = runtime.imu()
    if field == 'orientation':
        msg.orientation.w = value
    else:
        msg.orientation.w = 1.0
        msg.angular_velocity.x = value
    node.imu_callback(msg)
    result = node.sta_pub.messages[-1]
    assert [result.angular.x, result.angular.y, result.angular.z] == [0, 0, 0]
    assert node.last_imu_orientation is msg.orientation


def test_capture_requires_enabled_setpoints_and_a_valid_sample(stabilization, runtime):
    node = stabilization.StabilizationNode()
    initial = node.desired_orientation
    node.last_imu_orientation = runtime.quaternion(x=1.0, w=1.0)
    node.capture_callback(NS())
    assert node.desired_orientation is initial
    node.imu_stabilization_params['use_setpoints'] = True
    node.last_imu_orientation = None
    node.capture_callback(NS())
    assert node.desired_orientation is initial
    node.last_imu_orientation = runtime.quaternion(x=1.0, w=1.0)
    node.capture_callback(NS())
    assert node.desired_orientation.x == pytest.approx(2 ** -0.5)
    before = node.desired_orientation
    node.last_imu_orientation = runtime.quaternion()
    node.capture_callback(NS())
    assert node.desired_orientation is before


def test_callback_catches_processing_errors_and_publish_forwards_command(stabilization, runtime, monkeypatch):
    node = stabilization.StabilizationNode()
    monkeypatch.setattr(node, '_imu_to_twist_control', Mock(side_effect=ValueError('bad sample')))
    node.imu_callback(runtime.imu())
    node.logger.error.assert_called_once()
    assert not node.sta_pub.messages
    command = runtime.twist()
    command.angular.x = 0.25
    node.publish_stabilization_commands(command)
    assert node.sta_pub.messages[-1].angular.x == 0.25


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_cleanup(stabilization, runtime, error):
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            stabilization.main()
    else:
        stabilization.main()
    assert runtime.Node.instances[-1].destroyed
    runtime.ros.shutdown.assert_called_once_with()
