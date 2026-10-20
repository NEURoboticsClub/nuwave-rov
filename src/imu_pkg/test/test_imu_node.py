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
def imu_module(runtime, monkeypatch):
    driver = runtime.install('imu_pkg.imu_driver', ImuDriver=Mock())
    sensor = driver.ImuDriver.return_value
    sensor.read_data.return_value = {
        'linear_acceleration': {'accel_x': 1.0, 'accel_y': 2.0, 'accel_z': 3.0},
        'angular_velocity': {'gyro_x': 4.0, 'gyro_y': 5.0, 'gyro_z': 6.0},
        'magnetometer': {}, 'game_quaternion': {'real': 1.0, 'i': 0.0, 'j': 0.0, 'k': 0.0}}
    module = runtime.load('imu_pkg', 'imu_node.py')
    monkeypatch.setattr(module, 'time', NS(sleep=Mock(side_effect=[None, None, AssertionError('retry limit')])) )
    module.driver = driver
    module.sensor = sensor
    return module


def test_constructor_wires_default_driver_topic_and_rate(imu_module):
    node = imu_module.IMUPublisher()
    assert node.name == 'imu'
    imu_module.driver.ImuDriver.assert_called_once_with(7, 100)
    assert node.publisher_.topic == 'imu'
    assert node.timer.period == pytest.approx(0.01)
    assert node.timer.callback == node.timer_callback
    assert node._qm == pytest.approx([0.7071068, 0, 0, -0.7071068])
    assert node._qm_inv == pytest.approx([0.7071068, 0, 0, 0.7071068])


def test_constructor_retries_and_respects_overrides(imu_module, runtime):
    runtime.Node.overrides = {'addr': 1, 'sampling_rate': 50, 'q_mount': [0, 0, 0, 1]}
    imu_module.driver.ImuDriver.side_effect = [OSError('offline'), imu_module.sensor]
    node = imu_module.IMUPublisher()
    assert imu_module.driver.ImuDriver.call_count == 2
    imu_module.driver.ImuDriver.assert_called_with(1, 50)
    imu_module.time.sleep.assert_called_once_with(1.0)
    assert node.timer.period == pytest.approx(0.02)
    assert node._qm == pytest.approx([1, 0, 0, 0])
    node.logger.error.assert_called_once()


@pytest.mark.parametrize('left, right, expected', [
    ([1, 0, 0, 0], [0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5]),
    ([0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]),
    ([0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, -1]),
    ([0, 1, 0, 0], [0, 1, 0, 0], [-1, 0, 0, 0]),
])
def test_quaternion_product_has_correct_order(imu_module, left, right, expected):
    assert imu_module.IMUPublisher._quat_mul(left, right) == pytest.approx(expected)


@pytest.mark.parametrize('quaternion, vector, expected', [
    ([1, 0, 0, 0], [1, 2, 3], [1, 2, 3]),
    ([0, 0, 0, 1], [1, 2, 3], [-1, -2, 3]),
    ([2 ** -0.5, 0, 0, 2 ** -0.5], [1, 0, 0], [0, 1, 0]),
])
def test_vector_rotation_known_angles(imu_module, quaternion, vector, expected):
    assert imu_module.IMUPublisher._rotate_vec(quaternion, vector) == pytest.approx(expected)


def test_timer_publishes_driver_values_with_identity_mount(imu_module, runtime):
    runtime.Node.overrides = {'q_mount': [0, 0, 0, 1]}
    node = imu_module.IMUPublisher()
    node.now = 2.5
    node.timer_callback()
    msg = node.publisher_.messages[-1]
    assert msg.header.frame_id == 'imu_link'
    assert msg.header.stamp == 2_500_000_000
    assert list(vars(msg.orientation).values()) == [0.0, 0.0, 0.0, 1.0]
    assert [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z] == [4, 5, 6]
    assert [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z] == [1, 2, 3]


def test_mount_rotation_transforms_both_vectors_and_orientation(imu_module, runtime):
    half = 2 ** -0.5
    runtime.Node.overrides = {'q_mount': [0, 0, half, half]}
    imu_module.sensor.read_data.return_value['game_quaternion'] = {
        'real': half, 'i': 0, 'j': 0, 'k': half}
    node = imu_module.IMUPublisher()
    node.timer_callback()
    msg = node.publisher_.messages[-1]
    assert [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z] == pytest.approx([1, 0, 0, 0])
    assert [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z] == pytest.approx([-5, 4, 6])
    assert [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z] == pytest.approx([-2, 1, 3])


@pytest.mark.parametrize('failure', [OSError('offline'), {}])
def test_failed_or_incomplete_read_skips_publication_and_recovers(imu_module, failure):
    node = imu_module.IMUPublisher()
    good = imu_module.sensor.read_data.return_value
    imu_module.sensor.read_data.side_effect = [failure, good]
    node.timer_callback()
    assert not node.publisher_.messages
    node.logger.error.assert_called_once()
    node.timer_callback()
    assert len(node.publisher_.messages) == 1


def test_missing_driver_skips_cycle(imu_module):
    node = imu_module.IMUPublisher()
    node.imu = None
    node.timer_callback()
    imu_module.sensor.read_data.assert_not_called()
    assert not node.publisher_.messages


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_cleanup(imu_module, runtime, error):
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            imu_module.main()
    else:
        imu_module.main()
    assert runtime.Node.instances[-1].destroyed
    runtime.ros.shutdown.assert_called_once_with()
