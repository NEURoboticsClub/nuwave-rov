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
def cameras(runtime):
    return runtime.load('camera_pkg', 'mock_cameras_node.py')


def test_constructor_uses_four_compressed_topics_and_best_effort_qos(cameras):
    node = cameras.MockCameras()
    assert node.name == 'mock_cameras'
    assert (node.width, node.height, node.frame) == (640, 480, 0)
    assert [pub.topic for pub in node.publishers_] == [
        f'/camera_{index}/image/compressed' for index in range(4)]
    assert all(pub.qos.depth == 1 and pub.qos.reliability == 1 for pub in node.publishers_)
    assert node.timers[0].period == pytest.approx(1 / 30)
    assert node.timers[0].callback == node.tick


def test_tick_publishes_real_decodable_jpegs_with_headers(cameras):
    node = cameras.MockCameras(num_cameras=2, width=320, height=240, fps=20)
    node.now = 1.25
    node.tick()
    assert node.frame == 1
    assert node.timers[0].period == pytest.approx(0.05)
    for index, pub in enumerate(node.publishers_):
        msg = pub.messages[-1]
        assert msg.format == 'jpeg'
        assert msg.header.frame_id == f'camera_{index}'
        assert msg.header.stamp == 1_250_000_000
        decoded = cameras.cv2.imdecode(cameras.np.frombuffer(msg.data, dtype='uint8'), 1)
        assert decoded.shape == (240, 320, 3)
        assert decoded.dtype == cameras.np.uint8
        shade = 3 + index * 40
        expected = [shade, 255 - shade, shade // 2 + index * 30]
        cameras.np.testing.assert_allclose(decoded[-1, -1], expected, atol=5)
    assert node.publishers_[0].messages[-1].data != node.publishers_[1].messages[-1].data


def test_animation_changes_between_frames_and_wraps_color(cameras):
    node = cameras.MockCameras(num_cameras=1, width=320, height=240)
    node.frame = 84
    node.tick()
    first = node.publishers_[0].messages[-1].data
    node.tick()
    second = node.publishers_[0].messages[-1].data
    assert first != second
    decoded = cameras.cv2.imdecode(cameras.np.frombuffer(second, dtype='uint8'), 1)
    cameras.np.testing.assert_allclose(decoded[-1, -1], [2, 253, 1], atol=5)
    assert node.frame == 86


def test_failed_encoding_skips_only_that_camera_and_recovers(cameras, monkeypatch):
    node = cameras.MockCameras(num_cameras=2, width=64, height=48)
    encode = cameras.cv2.imencode
    attempts = iter([False, True, True, True])

    def controlled_encode(extension, frame):
        return encode(extension, frame) if next(attempts) else (False, None)

    monkeypatch.setattr(cameras.cv2, 'imencode', controlled_encode)
    node.tick()
    assert [len(pub.messages) for pub in node.publishers_] == [0, 1]
    node.tick()
    assert [len(pub.messages) for pub in node.publishers_] == [1, 2]


def test_zero_cameras_still_advances_frame_without_publishing(cameras):
    node = cameras.MockCameras(num_cameras=0)
    node.tick()
    assert node.frame == 1
    assert node.publishers_ == []


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_cleanup(cameras, runtime, error):
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            cameras.main()
    else:
        cameras.main()
    runtime.ros.init.assert_called_once_with()
    node = runtime.Node.instances[-1]
    runtime.ros.spin.assert_called_once_with(node)
    assert node.destroyed
    runtime.ros.shutdown.assert_called_once_with()
