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
def crab(runtime, tmp_path):
    runtime.Node.overrides = {'output_path': str(tmp_path / 'results' / 'result.jpg')}
    model = Mock()
    model.names = {0: 'green_crab', 1: 'other_crab'}
    model.return_value = [NS(boxes=[])]
    yolo = runtime.install('ultralytics', YOLO=Mock(return_value=model))
    module = runtime.load('crab_recognition_pkg', 'crab_recognition_node.py')
    module.model_factory = yolo.YOLO
    module.fake_model = model
    return module


def test_constructor_loads_model_and_captures_distinct_subscription_ids(crab, monkeypatch):
    node = crab.CrabRecognitionNode()
    assert node.name == 'crab_recognition'
    crab.model_factory.assert_called_once_with(crab.DEFAULT_MODEL_PATH)
    assert [sub.topic for sub in node.camera_subs] == [
        f'/camera_{index}/image/compressed' for index in range(4)]
    assert all(sub.qos.depth == 1 and sub.qos.reliability == 1 for sub in node.camera_subs)
    callback = Mock()
    monkeypatch.setattr(node, 'camera_callback', callback)
    message = NS(data=b'image')
    for sub in node.camera_subs:
        sub.callback(message)
    assert [entry.args[1] for entry in callback.call_args_list] == [0, 1, 2, 3]
    assert node.crab_scan_sub.topic == '/gui_buttons/detect_crabs'


@pytest.mark.parametrize('count, expected', [(0, 1), (2, 2), (10, 4)])
def test_camera_count_is_bounded_and_parameters_are_used(crab, runtime, count, expected):
    runtime.Node.overrides.update({'camera_count': count, 'camera_topic_prefix': '/test/camera_',
                                  'model_path': '/test/model.pt', 'confidence': 0.8})
    node = crab.CrabRecognitionNode()
    assert node.camera_count == len(node.camera_subs) == expected
    assert node.camera_subs[0].topic == '/test/camera_0/image/compressed'
    assert node.confidence == 0.8
    crab.model_factory.assert_called_once_with('/test/model.pt')


def test_camera_callback_decodes_real_jpeg_and_keeps_latest_frame(crab):
    node = crab.CrabRecognitionNode()
    for value in [50, 150]:
        frame = crab.np.full((24, 32, 3), value, dtype='uint8')
        ok, encoded = crab.cv2.imencode('.jpg', frame)
        assert ok
        node.camera_callback(NS(data=encoded.tobytes()), 2)
        crab.np.testing.assert_allclose(node.latest_frames[2], frame, atol=2)
    assert list(node.latest_frames) == [2]


@pytest.mark.parametrize('data', [b'not a JPEG', b''])
def test_invalid_jpeg_preserves_last_good_frame(crab, data):
    node = crab.CrabRecognitionNode()
    previous = crab.np.zeros((24, 32, 3), dtype='uint8')
    node.latest_frames[1] = previous
    node.camera_callback(NS(data=data), 1)
    assert node.latest_frames[1] is previous


def test_trigger_ignores_false_and_busy_then_resets_after_failure(crab, monkeypatch):
    node = crab.CrabRecognitionNode()
    detect = Mock(side_effect=RuntimeError('inference failed'))
    monkeypatch.setattr(node, 'run_detection_once', detect)
    node.crab_scan_callback(NS(data=False))
    node._busy = True
    node.crab_scan_callback(NS(data=True))
    detect.assert_not_called()
    node._busy = False
    node.crab_scan_callback(NS(data=True))
    detect.assert_called_once_with()
    assert not node._busy
    node.logger.error.assert_called_once()
    detect.side_effect = None
    node.crab_scan_callback(NS(data=True))
    assert detect.call_count == 2 and not node._busy


def test_no_frames_skip_model_inference(crab):
    node = crab.CrabRecognitionNode()
    node.run_detection_once()
    crab.fake_model.assert_not_called()
    node.logger.warning.assert_called_once()
    assert not list(Path(node.output_path).parent.glob('*'))


@pytest.mark.parametrize('suffix', ['.jpg', '.png', ''])
def test_detection_sorts_cameras_counts_only_green_crabs_and_writes_images(crab, suffix):
    node = crab.CrabRecognitionNode()
    node.output_path = str(Path(node.output_path).with_suffix(suffix))
    node.confidence = 0.8
    node.latest_frames = {2: crab.np.full((240, 320, 3), 140, dtype='uint8'),
                          0: crab.np.full((240, 320, 3), 80, dtype='uint8')}
    originals = {key: frame.copy() for key, frame in node.latest_frames.items()}
    crab.fake_model.return_value = [NS(boxes=[
        NS(cls=[0], conf=[0.9], xyxy=[[20.5, 30.5, 70.5, 80.5]]),
        NS(cls=[1], conf=[0.95], xyxy=[[90, 90, 120, 120]])])]
    node.run_detection_once()
    assert crab.fake_model.call_count == 2
    calls = crab.fake_model.call_args_list
    assert calls[0].kwargs == calls[1].kwargs == {'conf': 0.8, 'verbose': False}
    assert calls[0].args[0][0, 0, 0] == 80
    assert calls[1].args[0][0, 0, 0] == 140
    files = sorted(Path(node.output_path).parent.glob('*'))
    assert len(files) == 2
    assert files[0].name.endswith('_camera0' + (suffix or '.jpg'))
    assert files[1].name.endswith('_camera2' + (suffix or '.jpg'))
    for path in files:
        assert crab.cv2.imread(str(path)).shape == (240, 320, 3)
    for key, frame in node.latest_frames.items():
        crab.np.testing.assert_array_equal(frame, originals[key])
    node.logger.info.assert_any_call('Total green crabs across 2 camera frames: 2')


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_cleanup(crab, runtime, error):
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            crab.main()
    else:
        crab.main()
    assert runtime.Node.instances[-1].destroyed
    runtime.ros.shutdown.assert_called_once_with()
