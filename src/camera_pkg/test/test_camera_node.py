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
def camera(runtime, monkeypatch):
    module = runtime.load('camera_pkg', 'camera_node.py')
    capture = Mock()
    capture.isOpened.return_value = True
    capture.read.return_value = (False, None)
    monkeypatch.setattr(module.cv2, 'VideoCapture', Mock(return_value=capture))
    monkeypatch.setattr(module, 'threading', NS(Thread=Mock(), Lock=module.threading.Lock))
    monkeypatch.setattr(module, 'time', NS(sleep=Mock(side_effect=[None] * 20 + [AssertionError('retry limit')])))
    module.capture = capture
    return module


def test_constructor_configures_device_capture_thread_qos_and_timer(camera, runtime):
    runtime.Node.overrides = {'camera_id': 2, 'camera_device_path': ' /dev/test-camera ',
                              'width': 640, 'height': 480, 'fps': 20, 'jpeg_quality': 85}
    node = camera.FastCameraPublisher()
    assert node.name == 'camera'
    assert node.topic == '/camera_2/image/compressed'
    assert node.frame_id == 'camera_2'
    assert node.publisher.qos.depth == 1
    assert node.publisher.qos.reliability == 1
    camera.cv2.VideoCapture.assert_called_once_with('/dev/test-camera', camera.cv2.CAP_V4L2)
    settings = {entry.args[0]: entry.args[1] for entry in camera.capture.set.call_args_list}
    assert settings[camera.cv2.CAP_PROP_FRAME_WIDTH] == 640
    assert settings[camera.cv2.CAP_PROP_FRAME_HEIGHT] == 480
    assert settings[camera.cv2.CAP_PROP_FPS] == 20
    assert settings[camera.cv2.CAP_PROP_BUFFERSIZE] == 1
    camera.threading.Thread.assert_called_once_with(target=node.capture_loop, daemon=True)
    node.capture_thread.start.assert_called_once_with()
    assert node.timer.period == pytest.approx(0.05)
    assert node.timer.callback == node.publish_frame


@pytest.mark.parametrize('release_error', [None, OSError('already gone')])
def test_setup_releases_retries_failed_open_and_configures_replacement(camera, release_error):
    node = camera.FastCameraPublisher()
    old = node.cap
    old.release.side_effect = release_error
    closed, opened = Mock(), Mock()
    closed.isOpened.return_value = False
    opened.isOpened.return_value = True
    camera.cv2.VideoCapture.side_effect = [closed, OSError('open failed'), opened]
    node.setup_camera()
    assert node.cap is opened
    old.release.assert_called_once_with()
    closed.release.assert_called_once_with()
    assert camera.time.sleep.call_count == 2
    assert opened.set.call_count == 5
    assert node.logger.warning.call_count == (2 if release_error else 1)


def test_capture_loop_keeps_latest_frame_and_capture_timestamp(camera):
    import numpy as np
    node = camera.FastCameraPublisher()
    frames = [np.full((8, 10, 3), value, dtype='uint8') for value in [10, 20]]
    count = 0

    def read():
        nonlocal count
        node.now = count + 1.0
        frame = frames[count]
        count += 1
        if count == len(frames):
            node.running = False
        return True, frame

    camera.capture.read.side_effect = read
    node.capture_loop()
    assert node.frame is frames[-1]
    assert node.frame_stamp == 2_000_000_000
    camera.time.sleep.assert_not_called()


def test_capture_loop_reopens_only_after_eleven_consecutive_failures(camera, monkeypatch):
    node = camera.FastCameraPublisher()
    attempts = iter([OSError('read failed')] + [(False, None)] * 10)

    def read():
        item = next(attempts)
        if isinstance(item, Exception):
            raise item
        return item

    camera.capture.read.side_effect = read
    reopen = Mock(side_effect=lambda: setattr(node, 'running', False))
    monkeypatch.setattr(node, 'setup_camera', reopen)
    node.capture_loop()
    assert camera.capture.read.call_count == 11
    reopen.assert_called_once_with()
    assert camera.time.sleep.call_count == 10
    node.logger.warning.assert_called_once()


def test_successful_capture_resets_failure_count(camera, monkeypatch):
    import numpy as np
    node = camera.FastCameraPublisher()
    frame = np.zeros((8, 10, 3), dtype='uint8')
    attempts = iter([(False, None)] * 10 + [(True, frame)] + [(False, None)] * 10 + [(True, frame)])
    count = 0

    def read():
        nonlocal count
        count += 1
        if count == 22:
            node.running = False
        return next(attempts)

    camera.capture.read.side_effect = read
    reopen = Mock()
    monkeypatch.setattr(node, 'setup_camera', reopen)
    node.capture_loop()
    assert camera.capture.read.call_count == 22
    reopen.assert_not_called()


def test_publish_uses_real_jpeg_encoding_and_capture_time(camera):
    import numpy as np
    node = camera.FastCameraPublisher()
    node.publish_frame()
    assert not node.publisher.messages
    node.frame = np.full((24, 32, 3), 120, dtype='uint8')
    original = node.frame.copy()
    node.frame_stamp = 123
    node.publish_frame()
    msg = node.publisher.messages[-1]
    assert msg.header.stamp == 123
    assert msg.header.frame_id == 'camera_0'
    assert msg.format == 'jpeg'
    decoded = camera.cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1)
    assert decoded.shape == (24, 32, 3)
    np.testing.assert_allclose(decoded, original, atol=2)
    np.testing.assert_array_equal(node.frame, original)


@pytest.mark.parametrize('outcome', [(False, None), OSError('encoder failed')])
def test_encoding_failures_skip_publication(camera, monkeypatch, outcome):
    import numpy as np
    node = camera.FastCameraPublisher()
    node.frame = np.zeros((8, 10, 3), dtype='uint8')
    encoder = Mock(side_effect=outcome) if isinstance(outcome, Exception) else Mock(return_value=outcome)
    monkeypatch.setattr(camera.cv2, 'imencode', encoder)
    node.publish_frame()
    assert not node.publisher.messages
    if isinstance(outcome, Exception):
        node.logger.warning.assert_called_once()


@pytest.mark.parametrize('has_capture, release_error', [(True, None), (True, OSError('gone')), (False, None)])
def test_destroy_stops_thread_releases_capture_and_destroys_ros_node(camera, has_capture, release_error):
    node = camera.FastCameraPublisher()
    camera.capture.release.side_effect = release_error
    if not has_capture:
        node.cap = None
    node.destroy_node()
    assert not node.running and node.destroyed
    node.capture_thread.join.assert_called_once_with(timeout=1.0)
    assert camera.capture.release.call_count == int(has_capture)


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_cleanup(camera, runtime, error):
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            camera.main()
    else:
        camera.main()
    assert runtime.Node.instances[-1].destroyed
    runtime.ros.shutdown.assert_called_once_with()
