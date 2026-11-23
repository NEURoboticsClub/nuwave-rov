"""Run the real PWM node logic with controlled ROS and hardware boundaries."""

import importlib
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call

import pytest
import yaml


class Time:
    """Represent ROS timestamps and durations without a running ROS clock."""

    def __init__(self, seconds=0):
        self.nanoseconds = round(seconds * 1e9)

    def __sub__(self, other):
        return Time((self.nanoseconds - other.nanoseconds) / 1e9)

    def __gt__(self, other):
        return self.nanoseconds > other.nanoseconds


class FakeNode:
    """Record node wiring while allowing the production constructor to run."""

    def __init__(self, name, **kwargs):
        self.parameters = kwargs.get('parameters', {}).copy()
        self.logger = Mock()
        self.clock = Mock()
        self.clock.now.return_value = Time()
        self.subscriptions = []
        self.publishers = []
        self.destroyed = False

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)
        return self.get_parameter(name)

    def get_parameter(self, name):
        value = self.parameters[name]
        return SimpleNamespace(
            value=value,
            get_parameter_value=lambda: SimpleNamespace(string_value=value),
        )

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(subscription)
        return subscription

    def create_publisher(self, message_type, topic, qos):
        publisher = Mock(topic=topic)
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)

    def destroy_node(self):
        self.destroyed = True


@pytest.fixture
def pwm_module(monkeypatch):
    """Isolate imports so no ROS runtime or hardware dependency is accessed."""
    package_path = Path(__file__).parents[1]
    modules = {}

    def module(name, **attributes):
        item = ModuleType(name)
        item.__dict__.update(attributes)
        modules[name] = item
        return item

    ros = module('rclpy', init=Mock(), spin=Mock(), shutdown=Mock())
    module('rclpy.node', Node=FakeNode)
    module('rclpy.duration', Duration=Time)
    policies = SimpleNamespace(RELIABLE=1, KEEP_LAST=1, TRANSIENT_LOCAL=1)
    module('rclpy.qos', QoSProfile=SimpleNamespace,
           DurabilityPolicy=policies, ReliabilityPolicy=policies,
           HistoryPolicy=policies)
    module('std_msgs')
    module('std_msgs.msg', Float32=SimpleNamespace, Bool=SimpleNamespace)
    module('ament_index_python')
    module('ament_index_python.packages',
           get_package_share_directory=lambda name: str(package_path))
    module('nuwave_utils_pkg')
    module('nuwave_utils_pkg.file_helpers',
           load_yaml=lambda path: yaml.safe_load(Path(path).read_text()))
    package = module('_pwm_node_test')
    package.__path__ = [str(package_path / 'thruster_pkg')]
    module('_pwm_node_test.pwm_scribe_hat', PWMScribeHat=Mock())
    module('_pwm_node_test.pwm_scribe_arduino', PWMScribeArduino=Mock())
    for name, item in modules.items():
        monkeypatch.setitem(sys.modules, name, item)
    node_module = importlib.import_module('_pwm_node_test.pwm_node')
    monkeypatch.setattr(node_module, 'time', SimpleNamespace(sleep=Mock()))
    node_module.ros = ros
    yield node_module
    for name in list(sys.modules):
        if name.startswith('_pwm_node_test.'):
            sys.modules.pop(name, None)


@pytest.fixture
def make_node(pwm_module, tmp_path):
    """Give each node independent motor records and a deterministic clock."""
    def create(motors=None, **parameters):
        if motors is None:
            motors = [
                dict(topic='/thruster/test', channel=2, neutral_us=1530,
                     speed_min_us=1280, speed_max_us=1780,
                     precision_min_us=1430, precision_max_us=1630,
                     slew_us_per_s=750),
                dict(topic='/arm/test', channel=9, neutral_us=1500,
                     min_us=1400, max_us=1700, slew_us_per_s=0),
            ]
        config = tmp_path / 'motors.yaml'
        config.write_text(yaml.safe_dump({'motors': motors}))
        return pwm_module.PWMNode(parameters={
            'simulate': True, 'individual_motor_config': str(config),
            **parameters,
        })
    return create


def test_startup_uses_repository_motor_configuration(pwm_module):
    node = pwm_module.PWMNode(parameters={'simulate': True})
    assert len(node.motors) == len(node.motor_subs) == len(node.response_pubs) == 14
    assert node.period_us == 20000
    assert node.timer.period == pytest.approx(0.02)
    assert node.timer.callback == node.update
    assert node.precision_mode_sub.topic == '/controls/precision_mode'
    assert node.precision_mode_sub.qos.depth == 1
    for motor, publisher in zip(node.motors, node.response_pubs):
        assert motor['target_us'] == motor['current_us'] == motor['neutral_us']
        assert not motor['got_first_cmd'] and not motor['watchdog_tripped']
        assert publisher.topic == motor['topic'] + '/response_pwm'
    pwm_module.PWMScribeHat.assert_not_called()
    pwm_module.PWMScribeArduino.assert_not_called()


@pytest.mark.parametrize('hardware,arguments', [
    ('hat', (7, 0x40)), ('arduino', ('/dev/ttyACM0', [9, 10, 11, 12])),
])
def test_hardware_setup_and_arming(make_node, pwm_module, hardware, arguments):
    node = make_node(simulate=False, pwm_hardware=hardware)
    factory = (pwm_module.PWMScribeHat if hardware == 'hat'
               else pwm_module.PWMScribeArduino)
    factory.assert_called_once_with(*arguments)
    node.pwm.setup.assert_called_once_with(50)
    assert node.pwm.set_pwm.call_args_list == [call(2, 1530), call(9, 1500)]
    pwm_module.time.sleep.assert_called_once_with(2.0)


def test_hardware_setup_failure_keeps_node_running(make_node, pwm_module):
    pwm_module.PWMScribeHat.return_value.setup.side_effect = OSError('offline')
    node = make_node(simulate=False)
    assert node.pwm is None
    node.update()
    assert len(node.logger.error.call_args_list) == 1
    assert all(pub.publish.called for pub in node.response_pubs)


@pytest.mark.parametrize('motors,error', [
    ([], 'No motors'),
    ([dict(topic='bad', neutral_us=1500, min_us=1500, max_us=1700)],
     'invalid PWM range'),
    ([dict(topic='bad', neutral_us=1500, max_us=1700)], 'invalid PWM range'),
    ([dict(topic='bad', speed_min_us=1300)], 'must define all'),
    ([dict(topic='bad', neutral_us=1500, speed_min_us=1500, speed_max_us=1700,
           precision_min_us=1400, precision_max_us=1600)], 'invalid SPEED'),
    ([dict(topic='bad', neutral_us=1500, speed_min_us=1300, speed_max_us=1700,
           precision_min_us=1400, precision_max_us=1500)], 'invalid PRECISION'),
])
def test_invalid_motor_configuration_is_rejected(make_node, motors, error):
    with pytest.raises(ValueError, match=error):
        make_node(motors=motors)


@pytest.mark.parametrize('command,expected', [
    (-2, 1400), (-1, 1400), (-0.5, 1450), (0, 1500),
    (0.5, 1600), (1, 1700), (2, 1700),
])
def test_mapping_preserves_asymmetric_bounds(make_node, command, expected):
    node = make_node()
    motor = node.motors[1]
    assert node._map_dutycycle_to_pwm(command, motor) == expected
    assert node._map_pwm_to_dutycycle(expected, motor) == max(-1, min(1, command))


def test_subscriptions_route_commands_to_the_correct_motor(make_node):
    node = make_node()
    node.clock.now.return_value = Time(1)
    node.motor_subs[0].callback(SimpleNamespace(data=0.5))
    node.motor_subs[1].callback(SimpleNamespace(data=-0.5))
    assert [motor['target_us'] for motor in node.motors] == [1655, 1450]
    assert all(motor['last_msg_time'].nanoseconds == 1e9 for motor in node.motors)
    assert all(motor['got_first_cmd'] for motor in node.motors)


@pytest.mark.parametrize('command', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_commands_leave_motor_state_unchanged(make_node, command):
    node = make_node()
    node.logger.warn.reset_mock()
    before = node.motors[0].copy()
    node.clock.now.return_value = Time(1)
    node.cmd_callback(SimpleNamespace(data=command), before['topic'])
    assert node.motors[0] == before
    node.logger.warn.assert_called_once()


def test_unknown_topic_is_ignored(make_node):
    node = make_node()
    node.logger.warn.reset_mock()
    before = [motor.copy() for motor in node.motors]
    node.cmd_callback(SimpleNamespace(data=1), '/unknown')
    assert node.motors == before
    node.logger.warn.assert_called_once()


def test_precision_toggle_clamps_state_and_preserves_fixed_ranges(make_node):
    node = make_node()
    motor = node.motors[0]
    motor.update(target_us=1780, current_us=1280)
    node._precision_mode_callback(SimpleNamespace(data=True))
    assert (motor['min_us'], motor['max_us']) == (1430, 1630)
    assert (motor['target_us'], motor['current_us']) == (1630, 1430)
    assert (node.motors[1]['min_us'], node.motors[1]['max_us']) == (1400, 1700)
    log_count = node.logger.info.call_count
    node._precision_mode_callback(SimpleNamespace(data=True))
    assert node.logger.info.call_count == log_count
    node._precision_mode_callback(SimpleNamespace(data=False))
    assert (motor['min_us'], motor['max_us']) == (1280, 1780)
    assert (motor['target_us'], motor['current_us']) == (1630, 1430)


def test_precision_default_is_applied_at_startup(make_node):
    node = make_node(precision_mode_default=True)
    assert (node.motors[0]['min_us'], node.motors[0]['max_us']) == (1430, 1630)


def test_incomplete_motor_ranges_are_skipped_by_mode_helper(make_node):
    node = make_node()
    node.motors = [dict(neutral_us=1500)]
    node._apply_precision_mode_to_all_motors()
    assert node.motors == [dict(neutral_us=1500)]


@pytest.mark.parametrize('command,expected', [(1, 1605), (-1, 1455), (0.2, 1580)])
def test_slew_limits_both_directions_using_elapsed_time(make_node, command, expected):
    node = make_node()
    node.pwm = Mock()
    node.cmd_callback(SimpleNamespace(data=command), node.motors[0]['topic'])
    node.clock.now.return_value = Time(0.1)
    node.update()
    assert node.motors[0]['current_us'] == expected
    assert node.motors[0]['target_us'] == pytest.approx(1530 + 250 * command)
    node.pwm.set_pwm.assert_any_call(2, expected)
    assert node.response_pubs[0].publish.call_args.args[0].data == pytest.approx(
        (expected - 1530) / 250)


@pytest.mark.parametrize('elapsed,slew', [(0, 750), (0.1, 0)])
def test_zero_elapsed_time_or_disabled_slew_uses_target(make_node, elapsed, slew):
    node = make_node()
    node.motors[0]['slew_us_per_s'] = slew
    node.cmd_callback(SimpleNamespace(data=1), node.motors[0]['topic'])
    node.clock.now.return_value = Time(elapsed)
    node.update()
    assert node.motors[0]['current_us'] == 1780


def test_watchdog_boundary_ramps_to_neutral_and_recovers(make_node):
    node = make_node()
    node.logger.warn.reset_mock()
    motor = node.motors[0]
    node.cmd_callback(SimpleNamespace(data=1), motor['topic'])
    node.clock.now.return_value = Time(0.5)
    node.update()
    assert motor['current_us'] == 1780 and not motor['watchdog_tripped']
    node.clock.now.return_value = Time(0.6)
    node.update()
    assert motor['target_us'] == 1530 and motor['current_us'] == 1705
    assert motor['watchdog_tripped']
    node.logger.warn.assert_called_once()
    node.clock.now.return_value = Time(0.7)
    node.update()
    assert motor['current_us'] == 1630
    node.logger.warn.assert_called_once()
    node.cmd_callback(SimpleNamespace(data=-1), motor['topic'])
    node.clock.now.return_value = Time(0.8)
    node.update()
    assert not motor['watchdog_tripped']
    assert motor['target_us'] == 1280 and motor['current_us'] == 1555
    node.clock.now.return_value = Time(1.4)
    node.update()
    assert motor['current_us'] == 1530 and motor['watchdog_tripped']
    assert node.logger.warn.call_count == 2


def test_no_command_keeps_all_motors_neutral_without_timeout(make_node):
    node = make_node()
    node.logger.warn.reset_mock()
    node.clock.now.return_value = Time(10)
    node.update()
    assert all(motor['current_us'] == motor['neutral_us'] for motor in node.motors)
    assert all(pub.publish.call_args.args[0].data == 0 for pub in node.response_pubs)
    node.logger.warn.assert_not_called()


@pytest.mark.parametrize('failure', ['hardware', 'publisher'])
def test_one_motor_failure_does_not_block_other_motors(make_node, failure):
    node = make_node()
    node.pwm = Mock()
    if failure == 'hardware':
        node.pwm.set_pwm.side_effect = [OSError('write failed'), None]
    else:
        node.response_pubs[0].publish.side_effect = RuntimeError('publish failed')
    node.update()
    node.pwm.set_pwm.assert_any_call(9, 1500)
    node.response_pubs[1].publish.assert_called_once()
    node.logger.error.assert_called_once()


def test_write_all_forwards_microseconds_to_each_channel(make_node):
    node = make_node()
    node.pwm = Mock()
    node._write_all_us(1600.5)
    assert node.pwm.set_pwm.call_args_list == [call(2, 1600.5), call(9, 1600.5)]


def test_arming_continues_when_one_channel_fails(make_node, pwm_module):
    node = make_node()
    node.pwm = Mock()
    node.pwm.set_pwm.side_effect = [OSError('offline'), None]
    node._arm()
    assert node.pwm.set_pwm.call_args_list == [call(2, 1530), call(9, 1500)]
    assert pwm_module.time.sleep.call_count == 2


@pytest.mark.parametrize('failure', [None, 'write', 'shutdown', 'simulate'])
def test_shutdown_stops_all_channels_and_destroys_ros_node(make_node, failure):
    node = make_node()
    node.pwm = None if failure == 'simulate' else Mock()
    if failure == 'write':
        node.pwm.set_pwm.side_effect = [OSError('offline'), None]
    elif failure == 'shutdown':
        node.pwm.shutdown.side_effect = OSError('offline')
    node.destroy_node()
    assert node.destroyed
    if node.pwm is not None:
        assert node.pwm.set_pwm.call_args_list == [call(2, 1530), call(9, 1500)]
        node.pwm.shutdown.assert_called_once()
    assert node.logger.error.call_count == (1 if failure == 'write' else 0)


@pytest.mark.parametrize('spin_error', [None, KeyboardInterrupt(), RuntimeError('spin')])
@pytest.mark.parametrize('cleanup_error', [None, RuntimeError('cleanup')])
def test_main_always_cleans_up(pwm_module, monkeypatch, spin_error, cleanup_error):
    node = Mock()
    node.destroy_node.side_effect = cleanup_error
    monkeypatch.setattr(pwm_module, 'PWMNode', Mock(return_value=node))
    pwm_module.ros.spin.side_effect = spin_error
    if isinstance(spin_error, RuntimeError):
        with pytest.raises(RuntimeError, match='spin'):
            pwm_module.main()
    else:
        pwm_module.main()
    pwm_module.ros.init.assert_called_once()
    pwm_module.ros.spin.assert_called_once_with(node)
    node.destroy_node.assert_called_once()
    pwm_module.ros.shutdown.assert_called_once()
