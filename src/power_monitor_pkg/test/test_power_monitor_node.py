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
def power(runtime):
    driver = runtime.install('power_monitor_pkg.power_monitor_driver', INA226=Mock())
    sensor = driver.INA226.return_value
    sensor.read_all.return_value = {
        'bus_voltage': 12.5, 'current': -2.0, 'power': 25.0, 'shunt_voltage': -0.008}
    module = runtime.load('power_monitor_pkg', 'power_monitor_node.py')
    module.driver = driver
    module.sensor = sensor
    return module


def test_constructor_wires_default_topics_and_calibration(power):
    node = power.PowerMonitorPublisher()
    assert node.name == 'power_monitor'
    power.driver.INA226.assert_called_once_with(i2c_address=0x4F, bus_number=1)
    power.sensor.configure.assert_called_once_with()
    power.sensor.calibrate.assert_called_once_with(max_expected_current=17.0)
    assert [pub.topic for pub in node.publishers] == [
        'power_monitor/bus_voltage', 'power_monitor/current',
        'power_monitor/power', 'power_monitor/shunt_voltage']
    assert node.timer.period == pytest.approx(0.1)
    assert node.timer.callback == node.timer_callback


@pytest.mark.parametrize('check', ['rate', 'shunt', 'current'])
def test_parameter_overrides_control_bus_topics_rate_and_calibration(power, runtime, check):
    runtime.Node.overrides = {'i2c_address': 0x4C, 'i2c_bus': 7, 'topic': '/test/power',
                              'publish_rate_hz': 20.0, 'max_expected_current': 8.0,
                              'shunt_resistor': 0.01}
    node = power.PowerMonitorPublisher()
    power.driver.INA226.assert_called_once_with(i2c_address=0x4C, bus_number=7)
    assert node.pub_voltage.topic == '/test/power/bus_voltage'
    if check == 'rate':
        assert node.timer.period == pytest.approx(0.05)
    elif check == 'shunt':
        assert power.sensor.shunt_resistor == 0.01
    else:
        power.sensor.calibrate.assert_called_once_with(max_expected_current=8.0)


def test_measurements_publish_to_the_correct_topics(power):
    node = power.PowerMonitorPublisher()
    node.timer_callback()
    assert [pub.messages[-1].data for pub in node.publishers] == [12.5, -2.0, 25.0, -0.008]
    power.sensor.read_all.assert_called_once_with()
    assert not node.read_error_logged


def test_read_errors_are_suppressed_until_recovery(power):
    node = power.PowerMonitorPublisher()
    sample = power.sensor.read_all.return_value
    power.sensor.read_all.side_effect = [OSError('offline'), OSError('offline'), sample,
                                        OSError('offline again')]
    node.timer_callback()
    node.timer_callback()
    assert node.read_error_logged
    assert node.logger.error.call_count == 1
    assert all(not pub.messages for pub in node.publishers)
    node.timer_callback()
    assert not node.read_error_logged
    assert all(len(pub.messages) == 1 for pub in node.publishers)
    node.timer_callback()
    assert node.logger.error.call_count == 2
    assert node.read_error_logged


@pytest.mark.parametrize('stage', ['construct', 'configure', 'calibrate'])
def test_initialization_failures_have_context_and_original_cause(power, stage):
    error = OSError('I2C unavailable')
    if stage == 'construct':
        power.driver.INA226.side_effect = error
    else:
        getattr(power.sensor, stage).side_effect = error
    with pytest.raises(power.PowerMonitorInitError) as caught:
        power.PowerMonitorPublisher()
    assert caught.value.__cause__ is error


def test_main_handles_initialization_failure_without_spinning(power, runtime):
    power.driver.INA226.side_effect = OSError('offline')
    power.main(args=['--ros-args'])
    runtime.ros.init.assert_called_once_with(args=['--ros-args'])
    runtime.ros.spin.assert_not_called()
    runtime.ros.shutdown.assert_called_once_with()
    assert '0x4F on bus 1' in runtime.Node.instances[-1].logger.error.call_args.args[0]


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_cleanup(power, runtime, error):
    runtime.ros.spin.side_effect = error
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            power.main()
    else:
        power.main()
    node = runtime.Node.instances[-1]
    assert node.destroyed
    runtime.ros.spin.assert_called_once_with(node)
    runtime.ros.shutdown.assert_called_once_with()
