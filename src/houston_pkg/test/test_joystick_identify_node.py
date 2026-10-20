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
def identify(runtime, monkeypatch, tmp_path):
    runtime.Node.overrides = {'output_file': str(tmp_path / 'joystick.yaml')}
    module = runtime.load('houston_pkg', 'joystick_identify_node.py')
    monkeypatch.setattr(module, 'os', NS(system=Mock()))
    monkeypatch.setattr(module, 'time', NS(sleep=Mock()))
    return module


def feed_input(monkeypatch, answers):
    """Supply a bounded sequence instead of opening an interactive prompt."""
    values = iter(answers)

    def answer(prompt):
        value = next(values)
        if isinstance(value, BaseException):
            raise value
        return value

    monkeypatch.setattr('builtins.input', answer)


def test_constructor_and_callback_show_actual_axis_button_indexes(identify, capsys):
    node = identify.JoystickIdentify()
    assert node.name == 'joystick_identify'
    assert node.subscription.topic == '/joy'
    assert node.subscription.callback == node.joy_callback
    assert node.last_msg is None
    command = NS(axes=[-0.25, 0.5], buttons=[0, 1])
    node.joy_callback(command)
    assert node.last_msg is command
    identify.os.system.assert_called_once_with('clear')
    identify.time.sleep.assert_called_once_with(0.1)
    text = capsys.readouterr().out
    assert 'Axis 0: -0.25' in text and 'Axis 1: +0.50' in text
    assert 'Button 0: Released' in text and 'Button 1: Pressed' in text


def test_no_joystick_input_does_not_prompt_or_save(identify, monkeypatch, capsys):
    prompt = Mock(side_effect=AssertionError('unexpected prompt'))
    monkeypatch.setattr('builtins.input', prompt)
    node = identify.JoystickIdentify()
    node.interactive_setup()
    prompt.assert_not_called()
    assert not Path(node.output_file).exists()
    assert 'No joystick messages' in capsys.readouterr().out


def test_axis_defaults_are_written_as_loadable_yaml(identify, monkeypatch):
    node = identify.JoystickIdentify()
    node.last_msg = NS(axes=[0] * 6, buttons=[])
    answers = [value for index in range(6) for value in [str(index), 'n', '', '']]
    feed_input(monkeypatch, answers + ['n'])
    node.interactive_setup()
    saved = identify.yaml.safe_load(Path(node.output_file).read_text())
    assert saved == {'joystick': [
        {'axis': name, 'input': index, 'invert': False, 'sensitivity': 1.0, 'scale': 'linear'}
        for index, name in enumerate(node.axis_names)]}


def test_custom_axes_and_multiple_buttons_round_trip(identify, monkeypatch):
    node = identify.JoystickIdentify()
    node.last_msg = NS(axes=[0] * 6, buttons=[0] * 4)
    answers = [value for index in range(6) for value in [str(index), ' Y ', '0.5', 'logarithmic']]
    feed_input(monkeypatch, answers + ['y', ' A ', '1', 'y', 'y', 'Start', '3', 'n', 'n'])
    node.interactive_setup()
    saved = identify.yaml.safe_load(Path(node.output_file).read_text())['joystick']
    assert all(item['invert'] and item['sensitivity'] == 0.5 and item['scale'] == 'logarithmic'
               for item in saved[:6])
    assert saved[6:] == [{'button': 'A', 'input': 1, 'invert': True},
                         {'button': 'Start', 'input': 3, 'invert': False}]


@pytest.mark.parametrize('answers', [['bad'], ['0', 'n', 'bad'], [KeyboardInterrupt()]])
def test_invalid_or_cancelled_axis_setup_does_not_save(identify, monkeypatch, answers):
    node = identify.JoystickIdentify()
    node.last_msg = NS(axes=[0], buttons=[])
    feed_input(monkeypatch, answers)
    node.interactive_setup()
    assert not Path(node.output_file).exists()


@pytest.mark.parametrize('answer', ['bad', KeyboardInterrupt()])
def test_invalid_button_input_saves_completed_axis_config(identify, monkeypatch, answer):
    node = identify.JoystickIdentify()
    node.last_msg = NS(axes=[0] * 6, buttons=[0])
    answers = [value for index in range(6) for value in [str(index), 'n', '', '']]
    feed_input(monkeypatch, answers + ['y', 'A', answer])
    node.interactive_setup()
    saved = identify.yaml.safe_load(Path(node.output_file).read_text())
    assert len(saved['joystick']) == 6
    assert all('axis' in item for item in saved['joystick'])


def test_save_config_reports_filesystem_errors(identify, tmp_path):
    node = identify.JoystickIdentify()
    node.output_file = str(tmp_path)
    with pytest.raises(IsADirectoryError):
        node.save_config()


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_always_cleans_up_after_spin(identify, runtime, monkeypatch, error):
    setup = Mock()
    monkeypatch.setattr(identify.JoystickIdentify, 'interactive_setup', setup)
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            identify.main()
        setup.assert_not_called()
    else:
        identify.main()
        setup.assert_called_once_with()
    assert runtime.Node.instances[-1].destroyed
    runtime.ros.shutdown.assert_called_once_with()


@pytest.mark.parametrize('error', [KeyboardInterrupt(), OSError('save failed')])
def test_main_cleans_up_when_wizard_is_cancelled_or_save_fails(identify, runtime, monkeypatch, error):
    monkeypatch.setattr(identify.JoystickIdentify, 'interactive_setup', Mock(side_effect=error))
    with pytest.raises(type(error)):
        identify.main()
    assert runtime.Node.instances[-1].destroyed
    runtime.ros.shutdown.assert_called_once_with()
