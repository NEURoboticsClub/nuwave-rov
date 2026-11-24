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
def arm(runtime):
    return runtime.load('arm_controller_pkg', 'arm_controller_node.py')


def test_constructor_loads_real_config_and_wires_motor_topics(arm):
    node = arm.ArmController()
    assert node.name == 'arm_controller'
    assert len(node.motors) == len(node.arm_motor_pubs) == 6
    assert [motor['id'] for motor in node.motors] == [1, 2, 3, 4, 5, 6]
    assert [pub.topic for pub in node.arm_motor_pubs] == [
        '/arm/motor_base_yaw', '/arm/motor_base_pitch', '/arm/motor_elbow_pitch',
        '/arm/motor_wrist_yaw', '/arm/motor_wrist_pitch', '/arm/motor_claw']
    assert all(motor['dutycycle'] == 0 for motor in node.motors)
    assert node.status_sub.topic == 'arm_commands'
    assert node.status_sub.callback == node.Status_Callback
    assert node.timers[0].period == pytest.approx(0.02)
    assert node.timers[0].callback == node.publish_arm_motors
    node.publish_arm_motors()
    assert [pub.messages[-1].data for pub in node.arm_motor_pubs] == [0.0] * 6


def test_parameter_overrides_and_empty_motor_config(arm, runtime, monkeypatch):
    runtime.Node.overrides = {'publish_rate_hz': 20, 'arm_topic': '/test/arm',
                              'watchdog_timeout_s': 1.0}
    monkeypatch.setattr(arm, 'load_yaml', lambda path: {})
    node = arm.ArmController()
    assert node.motors == node.arm_motor_pubs == []
    assert node.status_sub.topic == '/test/arm'
    assert node.timers[0].period == pytest.approx(0.05)
    assert node.watchdog_timeout_s == 1.0
    node.Status_Callback(NS(data=[]))
    node.publish_arm_motors()


def test_callback_maps_motor_ids_independently_of_config_order(arm, monkeypatch):
    monkeypatch.setattr(arm, 'load_yaml', lambda path: {
        'arm_motors': [{'id': 3, 'topic': '/third'}, {'id': 1, 'topic': '/first'}]})
    node = arm.ArmController()
    command = NS(data=[-0.5, 0.0, 0.75])
    node.now = 3.0
    node.Status_Callback(command)
    node.publish_arm_motors()
    assert node.last_state_msg is command
    assert node.last_msg_time.nanoseconds == 3_000_000_000
    assert [pub.messages[-1].data for pub in node.arm_motor_pubs] == [0.75, -0.5]


@pytest.mark.parametrize('values, expected', [
    ([], [0.0] * 6), ([0.5, -0.5], [0.5, -0.5, 0.0, 0.0, 0.0, 0.0]),
    ([float('nan'), float('inf'), -float('inf'), 1, -1, 0], [0, 0, 0, 1, -1, 0]),
    ([1, 2, 3, 4, 5, 6, 7], [1, 2, 3, 4, 5, 6]),
])
def test_missing_and_nonfinite_commands_are_zeroed(arm, values, expected):
    node = arm.ArmController()
    node.Status_Callback(NS(data=values))
    node.publish_arm_motors()
    assert [pub.messages[-1].data for pub in node.arm_motor_pubs] == expected


@pytest.mark.parametrize('age, expected', [(0.499999999, 0.75), (0.5, 0.75),
                                          (0.500000001, 0.0)])
def test_watchdog_boundary(arm, age, expected):
    node = arm.ArmController()
    node.Status_Callback(NS(data=[0.75] * 6))
    node.now = age
    node.publish_arm_motors()
    assert [pub.messages[-1].data for pub in node.arm_motor_pubs] == [expected] * 6


def test_watchdog_warns_once_then_rearms_after_recovery(arm):
    node = arm.ArmController()
    node.Status_Callback(NS(data=[0.5] * 6))
    node.now = 1.0
    node.publish_arm_motors()
    node.publish_arm_motors()
    assert node.watchdog_tripped
    assert node.logger.warn.call_count == 1
    node.Status_Callback(NS(data=[-0.25] * 6))
    node.publish_arm_motors()
    assert not node.watchdog_tripped
    assert [pub.messages[-1].data for pub in node.arm_motor_pubs] == [-0.25] * 6
    node.now = 2.0
    node.publish_arm_motors()
    assert node.logger.warn.call_count == 2


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_cleanup(arm, runtime, error):
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            arm.main(args=['--ros-args'])
    else:
        arm.main(args=['--ros-args'])
    runtime.ros.init.assert_called_once_with(args=['--ros-args'])
    node = runtime.Node.instances[-1]
    runtime.ros.spin.assert_called_once_with(node)
    assert node.destroyed
    runtime.ros.shutdown.assert_called_once_with()
