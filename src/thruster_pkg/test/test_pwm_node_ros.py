"""Exercise PWMNode with real ROS 2 transport, clocks, timers and messages.

Only the hardware adapters and arming sleep are replaced. Run in a sourced
ROS workspace; the helper suite in test_pwm_node.py also runs without ROS.
"""

import importlib
import os
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call

import pytest
import yaml

pytest.importorskip('rclpy', reason='Source a ROS 2 installation for transport tests')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32


@pytest.fixture
def ros_pwm(monkeypatch, tmp_path):
    """Use an isolated ROS domain and a recording adapter, never a device."""
    package = ModuleType('_pwm_ros_test')
    package.__path__ = [str(Path(__file__).parents[1] / 'thruster_pkg')]
    monkeypatch.setitem(sys.modules, package.__name__, package)
    adapters = []
    for name, factory in [('hat', 'PWMScribeHat'), ('arduino', 'PWMScribeArduino')]:
        module = ModuleType(f'{package.__name__}.pwm_scribe_{name}')
        adapter = Mock()
        setattr(module, factory, Mock(return_value=adapter))
        monkeypatch.setitem(sys.modules, module.__name__, module)
        adapters.append(adapter)
    pwm = importlib.import_module(f'{package.__name__}.pwm_node')
    monkeypatch.setattr(pwm, 'time', SimpleNamespace(sleep=Mock()))
    context = Context()
    context.init(domain_id=40 + os.getpid() % 150)
    executor = SingleThreadedExecutor(context=context)
    peer = Node('pwm_test_peer', context=context)
    executor.add_node(peer)
    nodes = []
    motors = [
        dict(topic='/kan16/thruster', channel=2, neutral_us=1530,
             speed_min_us=1280, speed_max_us=1780,
             precision_min_us=1430, precision_max_us=1630, slew_us_per_s=0.0),
        dict(topic='/kan16/arm', channel=9, neutral_us=1500,
             min_us=1400, max_us=1700, slew_us_per_s=0.0),
    ]

    def create(**parameters):
        config = tmp_path / 'motors.yaml'
        config.write_text(yaml.safe_dump({'motors': motors}))
        parameters = {'individual_motor_config': str(config), **parameters}
        node = pwm.PWMNode(
            context=context,
            parameter_overrides=[Parameter(key, value=value)
                                 for key, value in parameters.items()],
        )
        nodes.append(node)
        executor.add_node(node)
        return node

    def until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        pytest.fail('Timed out waiting for ROS discovery or callback output')

    yield SimpleNamespace(create=create, peer=peer, until=until, motors=motors,
                          adapter=adapters[0], executor=executor, nodes=nodes)
    for node in nodes:
        executor.remove_node(node)
        node.destroy_node()
    executor.remove_node(peer)
    peer.destroy_node()
    executor.shutdown()
    context.shutdown()
    for name in list(sys.modules):
        if name.startswith('_pwm_ros_test.'):
            sys.modules.pop(name, None)


@pytest.mark.parametrize('simulate', [False, True])
def test_commands_route_clamp_and_publish_feedback(ros_pwm, simulate):
    """Send Float32s through DDS and inspect timer-driven hardware and feedback."""
    ros = ros_pwm
    node = ros.create(watchdog_timeout_s=10.0, simulate=simulate)
    responses = [[], []]
    subscriptions = [
        ros.peer.create_subscription(
            Float32, motor['topic'] + '/response_pwm',
            lambda msg, index=index: responses[index].append(msg.data), 10)
        for index, motor in enumerate(ros.motors)
    ]
    publishers = [ros.peer.create_publisher(Float32, motor['topic'], 10)
                  for motor in ros.motors]
    ros.until(lambda: all(pub.get_subscription_count() for pub in publishers)
              and all(sub.get_publisher_count() for sub in subscriptions))
    publishers[0].publish(Float32(data=2.0))
    publishers[1].publish(Float32(data=-0.5))
    ros.until(lambda: responses[0] and responses[1]
              and responses[0][-1] == 1.0 and responses[1][-1] == -0.5)
    assert [motor['current_us'] for motor in node.motors] == [1780, 1450]
    if simulate:
        ros.adapter.set_pwm.assert_not_called()
    else:
        ros.adapter.set_pwm.assert_any_call(2, 1780)
        ros.adapter.set_pwm.assert_any_call(9, 1450)
    publishers[0].publish(Float32(data=0.5))
    ros.until(lambda: responses[0][-1] == 0.5)
    assert node.motors[0]['target_us'] == 1655


def test_precision_mode_receives_retained_state(ros_pwm):
    """A late PWM subscriber must receive the operator's latched mode."""
    ros = ros_pwm
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = ros.peer.create_publisher(Bool, '/controls/precision_mode', qos)
    publisher.publish(Bool(data=True))
    node = ros.create()
    ros.until(lambda: node.precision_mode_enabled)
    motor = node.motors[0]
    assert (motor['min_us'], motor['max_us']) == (1430, 1630)
    assert (node.motors[1]['min_us'], node.motors[1]['max_us']) == (1400, 1700)
    publisher.publish(Bool(data=False))
    ros.until(lambda: not node.precision_mode_enabled)
    assert (motor['min_us'], motor['max_us']) == (1280, 1780)


def test_watchdog_stops_and_recovers_through_ros(ros_pwm):
    """The ROS timer returns to neutral when a real command stream stops."""
    ros = ros_pwm
    node = ros.create(watchdog_timeout_s=0.3)
    received = []
    ros.peer.create_subscription(Float32, '/kan16/thruster/response_pwm',
                                 lambda msg: received.append(msg.data), 10)
    publisher = ros.peer.create_publisher(Float32, '/kan16/thruster', 10)
    ros.until(lambda: publisher.get_subscription_count() and received)
    publisher.publish(Float32(data=1.0))
    ros.until(lambda: received[-1] == 1.0)
    ros.until(lambda: node.motors[0]['watchdog_tripped'] and received[-1] == 0.0)
    ros.adapter.set_pwm.assert_any_call(2, 1530)
    publisher.publish(Float32(data=-1.0))
    ros.until(lambda: received[-1] == -1.0)
    assert not node.motors[0]['watchdog_tripped']


def test_slew_uses_ros_clock_and_shutdown_sends_neutral(ros_pwm):
    """Measure the first timer step and verify neutral writes on ROS teardown."""
    ros = ros_pwm
    ros.motors[0]['slew_us_per_s'] = 100.0
    node = ros.create(watchdog_timeout_s=10.0)
    publisher = ros.peer.create_publisher(Float32, '/kan16/thruster', 10)
    ros.until(lambda: publisher.get_subscription_count() > 0)
    motor = node.motors[0]
    previous_time = motor['prev_time']
    publisher.publish(Float32(data=1.0))
    ros.until(lambda: motor['current_us'] > 1530)
    elapsed = (motor['prev_time'] - previous_time).nanoseconds / 1e9
    assert 1530 < motor['current_us'] < 1780
    assert motor['current_us'] - 1530 <= 100.0 * elapsed + 1e-6
    ros.adapter.set_pwm.reset_mock()
    node.destroy_node()
    assert ros.adapter.set_pwm.call_args_list == [call(2, 1530), call(9, 1500)]
    ros.adapter.shutdown.assert_called_once()
    ros.executor.remove_node(node)
    ros.nodes.remove(node)
