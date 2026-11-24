"""Exercise the real depth node with ROS and the I2C sensor replaced at the boundary."""

import copy
import importlib
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call

import pytest


class FakeNode:
    """Record ROS wiring and published message snapshots without running ROS."""

    def __init__(self, name):
        self.name = name
        self.parameters = {}
        self.logger = Mock()
        self.stamp = object()
        self.clock = SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: self.stamp))
        self.publishers = {}
        self.destroy_node = Mock()

    def declare_parameter(self, name, default):
        self.parameters[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock

    def create_publisher(self, message_type, topic, qos):
        publisher = SimpleNamespace(message_type=message_type, qos=qos, messages=[])
        publisher.publish = lambda message: publisher.messages.append(copy.deepcopy(message))
        self.publishers[topic] = publisher
        return publisher

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)


def stamped_message():
    """Provide the header fields used by FluidPressure and Temperature."""
    return SimpleNamespace(header=SimpleNamespace(stamp=None, frame_id=''))


@pytest.fixture
def depth_module(monkeypatch):
    """Import production code under a private package; never load the I2C driver."""
    modules = {}

    def module(name, **attributes):
        item = ModuleType(name)
        item.__dict__.update(attributes)
        modules[name] = item
        return item

    ros = module('rclpy', init=Mock(), spin=Mock(), shutdown=Mock())
    module('rclpy.node', Node=FakeNode)
    module('sensor_msgs')
    module('sensor_msgs.msg', FluidPressure=stamped_message, Temperature=stamped_message)
    module('std_msgs')
    module('std_msgs.msg', Float32=SimpleNamespace, Float32MultiArray=SimpleNamespace)
    package = module('_depth_sensor_test')
    package.__path__ = [str(Path(__file__).parents[1] / 'depth_sensor_pkg')]
    sensor = Mock(spec=['init', 'read', 'depth', 'pressure', 'temperature'])
    sensor.init.return_value = True
    sensor.read.return_value = True
    sensor.depth.return_value = 2.75
    sensor.pressure.return_value = 1234.5
    sensor.temperature.return_value = 18.25
    driver = module('_depth_sensor_test.ms5837_driver', MS5837_02BA=Mock(return_value=sensor))
    for name, item in modules.items():
        monkeypatch.setitem(sys.modules, name, item)
    node_module = importlib.import_module('_depth_sensor_test.depth_sensor_node')
    monkeypatch.setattr(node_module, 'time', SimpleNamespace(sleep=Mock()))
    node_module.ros = ros
    node_module.fake_sensor = sensor
    node_module.fake_driver = driver
    try:
        yield node_module
    finally:
        sys.modules.pop('_depth_sensor_test.depth_sensor_node', None)


def test_constructor_wires_sensor_publishers_and_timer(depth_module):
    node = depth_module.DepthSensorNode()

    assert node.name == 'depth_sensor'
    assert node.parameters == {'publish_rate': 10.0, 'depth_list_length': 10, 'i2c_bus': 1}
    assert node.depth_list == []
    assert node.depth_list_length == 10
    depth_module.fake_driver.MS5837_02BA.assert_called_once_with(1)
    depth_module.fake_sensor.init.assert_called_once_with()
    depth_module.time.sleep.assert_not_called()
    assert set(node.publishers) == {
        'depth/depth', 'depth/depth_array', 'depth/pressure', 'depth/temperature'}
    assert node.depth_pub.message_type is depth_module.Float32
    assert node.depth_array_pub.message_type is depth_module.Float32MultiArray
    assert node.pressure_pub.message_type is depth_module.FluidPressure
    assert node.temperature_pub.message_type is depth_module.Temperature
    assert all(publisher.qos == 10 for publisher in node.publishers.values())
    assert node.timer.period == pytest.approx(0.1)
    assert node.timer.callback == node.read_and_publish
    assert all(not publisher.messages for publisher in node.publishers.values())


def test_constructor_retries_failed_and_raising_sensor_initialization(depth_module):
    depth_module.fake_sensor.init.side_effect = [False, OSError('I2C unavailable'), True]
    # Bound retries even if a regression makes initialization loop indefinitely.
    depth_module.time.sleep.side_effect = [None, None, AssertionError('unexpected retry')]

    node = depth_module.DepthSensorNode()

    assert depth_module.fake_sensor.init.call_count == 3
    assert depth_module.time.sleep.call_args_list == [call(1.0), call(1.0)]
    assert node.logger.error.call_count == 3
    assert any('I2C unavailable' in entry.args[0] for entry in node.logger.error.call_args_list)
    assert node.timer.callback == node.read_and_publish


@pytest.mark.parametrize('depth', [0.0, -0.25, 2.75])
def test_read_depth_returns_sensor_value(depth_module, depth):
    node = depth_module.DepthSensorNode()
    depth_module.fake_sensor.depth.return_value = depth

    assert node.read_depth() == depth

    depth_module.fake_sensor.read.assert_called_once_with()
    assert all(not publisher.messages for publisher in node.publishers.values())


def test_read_depth_raises_on_failed_read(depth_module):
    node = depth_module.DepthSensorNode()
    depth_module.fake_sensor.read.return_value = False

    with pytest.raises(LookupError, match='error reading from depth sensor'):
        node.read_depth()

    depth_module.fake_sensor.depth.assert_not_called()
    depth_module.fake_sensor.pressure.assert_not_called()
    depth_module.fake_sensor.temperature.assert_not_called()
    node.logger.error.assert_called_once_with('Sensor read failed!')


def test_read_and_publish_values_units_and_headers(depth_module):
    node = depth_module.DepthSensorNode()
    node.stamp = 123456

    node.read_and_publish()

    assert node.depth_pub.messages[0].data == pytest.approx(2.75)
    assert node.depth_array_pub.messages[0].data == [2.75]
    pressure = node.pressure_pub.messages[0]
    temperature = node.temperature_pub.messages[0]
    assert pressure.fluid_pressure == pytest.approx(123450.0)
    assert temperature.temperature == pytest.approx(18.25)
    for message in (pressure, temperature):
        assert message.header.stamp == node.stamp
        assert message.header.frame_id == 'depth_sensor'
        assert message.variance == 0.0
    assert all(len(publisher.messages) == 1 for publisher in node.publishers.values())
    node.logger.error.assert_not_called()


@pytest.mark.parametrize('window', [1, 3, 10])
def test_depth_history_keeps_exact_configured_window(depth_module, window):
    node = depth_module.DepthSensorNode()
    node.depth_list_length = window
    samples = [float(value) for value in range(window + 3)]

    for index, depth in enumerate(samples):
        depth_module.fake_sensor.depth.return_value = depth
        node.read_and_publish()

        expected = samples[max(0, index + 1 - window):index + 1]
        assert node.depth_list == expected
        assert node.depth_array_pub.messages[-1].data == expected
        assert node.depth_pub.messages[-1].data == depth

    # Previously published arrays must remain snapshots of that publication.
    assert node.depth_array_pub.messages[0].data == [0.0]
    node.logger.error.assert_not_called()


@pytest.mark.parametrize('failure', [False, OSError('I2C read failed')])
def test_failed_read_preserves_history_and_recovers(depth_module, failure):
    node = depth_module.DepthSensorNode()
    node.read_and_publish()
    previous_history = node.depth_list.copy()
    depth_module.fake_sensor.read.side_effect = [failure, True]

    node.read_and_publish()

    assert node.depth_list == previous_history
    assert all(len(publisher.messages) == 1 for publisher in node.publishers.values())
    assert 'Error in depth sensor:' in node.logger.error.call_args.args[0]

    depth_module.fake_sensor.depth.return_value = 3.5
    node.read_and_publish()

    assert node.depth_list == [2.75, 3.5]
    assert all(len(publisher.messages) == 2 for publisher in node.publishers.values())
    assert node.depth_pub.messages[-1].data == 3.5


@pytest.mark.parametrize('spin_error', [None, KeyboardInterrupt(), RuntimeError('spin failed')])
def test_main_spins_and_cleans_up(depth_module, monkeypatch, spin_error):
    node = depth_module.DepthSensorNode()
    constructor = Mock(return_value=node)
    monkeypatch.setattr(depth_module, 'DepthSensorNode', constructor)
    depth_module.ros.spin.side_effect = spin_error
    args = ['--ros-args']

    if isinstance(spin_error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin failed'):
            depth_module.main(args=args)
    else:
        depth_module.main(args=args)

    depth_module.ros.init.assert_called_once_with(args=args)
    constructor.assert_called_once_with()
    depth_module.ros.spin.assert_called_once_with(node)
    node.destroy_node.assert_called_once_with()
    depth_module.ros.shutdown.assert_called_once_with()
