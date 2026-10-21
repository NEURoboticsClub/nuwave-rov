"""Exercise the thruster controller through its real ROS topics."""

import time

from geometry_msgs.msg import Twist
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

LATERAL = range(0, 4)
VERTICAL = range(4, 8)


@pytest.fixture(scope='module')
def ros_context():
    """Init/shutdown rclpy once for the whole test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def executor(ros_context):
    """Create a fresh executor per test."""
    exc = SingleThreadedExecutor()
    yield exc
    exc.shutdown()


@pytest.fixture
def controller_node(executor):
    """Spin up the real ThrusterController node with the package config."""
    from thruster_controller_pkg.thruster_controller_node import ThrusterController
    node = ThrusterController()
    executor.add_node(node)
    yield node
    node.destroy_node()


class ThrusterListener(Node):
    """Subscribe to every configured thruster topic and record the commands."""

    def __init__(self, topics):
        super().__init__('test_thruster_listener')
        self.received = {i: [] for i in range(len(topics))}
        self.subs = [
            self.create_subscription(Float32, topic, self._make_cb(i), 10)
            for i, topic in enumerate(topics)
        ]

    def _make_cb(self, index):
        def cb(msg):
            self.received[index].append(msg.data)
        return cb

    def latest(self):
        """Return the most recent command per thruster, or None if nothing yet."""
        return {i: msgs[-1] if msgs else None for i, msgs in self.received.items()}


@pytest.fixture
def listener(executor, controller_node):
    """Listen on the topics the controller was configured with."""
    node = ThrusterListener([t['topic'] for t in controller_node.thrusters])
    executor.add_node(node)
    yield node
    node.destroy_node()


class TwistPublisher(Node):
    """Publish Twist commands on the controller's input topic."""

    def __init__(self):
        super().__init__('test_twist_publisher')
        self.pub = self.create_publisher(Twist, 'velocity_commands', 10)

    def send(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = [float(v) for v in linear]
        msg.angular.x, msg.angular.y, msg.angular.z = [float(v) for v in angular]
        self.pub.publish(msg)


@pytest.fixture
def publisher(executor):
    """Provide a Twist publisher on the executor."""
    node = TwistPublisher()
    executor.add_node(node)
    yield node
    node.destroy_node()


def spin_for(executor, seconds, step=0.01):
    """Spin the executor for a fixed duration to let messages flow."""
    end = time.time() + seconds
    while time.time() < end:
        executor.spin_once(timeout_sec=step)


def send_and_read(executor, publisher, listener, linear=(0, 0, 0), angular=(0, 0, 0)):
    """Send one Twist, let the controller publish, and return the latest commands."""
    publisher.send(linear=linear, angular=angular)
    spin_for(executor, 0.3)
    latest = listener.latest()
    assert all(v is not None for v in latest.values()), f'Missing commands: {latest}'
    return latest


class TestNodeStartup:

    def test_one_publisher_per_configured_thruster(self, controller_node):
        """Create one publisher for each thruster in the config."""
        assert len(controller_node.thrusters) == 8
        assert len(controller_node.thruster_pubs) == len(controller_node.thrusters)

    def test_publishes_zero_without_input(self, executor, controller_node, listener):
        """Publish 0.0 on every thruster before any velocity command arrives."""
        spin_for(executor, 0.3)
        assert all(v == 0.0 for v in listener.latest().values())


class TestDirectionalResponse:

    def test_zero_twist_gives_zero(self, executor, controller_node, publisher, listener):
        """A zero Twist commands every thruster to 0.0."""
        latest = send_and_read(executor, publisher, listener)
        np.testing.assert_allclose(list(latest.values()), 0.0, atol=1e-6)

    def test_pure_heave_drives_vertical_only(
            self, executor, controller_node, publisher, listener):
        """A pure Z command moves only the vertical thrusters, all upward."""
        latest = send_and_read(executor, publisher, listener, linear=(0, 0, 0.5))
        for i in LATERAL:
            assert abs(latest[i]) < 1e-3, f'Lateral thruster {i} moved: {latest[i]}'
        for i in VERTICAL:
            assert latest[i] > 0.1, f'Vertical thruster {i} did not push up: {latest[i]}'

    def test_pure_surge_drives_lateral_only(
            self, executor, controller_node, publisher, listener):
        """A pure Y command moves only the lateral thrusters."""
        latest = send_and_read(executor, publisher, listener, linear=(0, 0.5, 0))
        for i in VERTICAL:
            assert abs(latest[i]) < 1e-3, f'Vertical thruster {i} moved: {latest[i]}'
        assert any(abs(latest[i]) > 0.1 for i in LATERAL)

    def test_opposite_commands_give_opposite_outputs(
            self, executor, controller_node, publisher, listener):
        """Forward and reverse commands produce mirrored thruster commands."""
        fwd = send_and_read(executor, publisher, listener, linear=(0.3, 0.3, 0))
        rev = send_and_read(executor, publisher, listener, linear=(-0.3, -0.3, 0))
        for i in fwd:
            assert abs(fwd[i] + rev[i]) < 1e-3, f'Thruster {i}: fwd={fwd[i]}, rev={rev[i]}'


class TestSafety:

    def test_large_input_is_clamped(self, executor, controller_node, publisher, listener):
        """Extreme input never commands beyond [-1, 1]."""
        latest = send_and_read(
            executor, publisher, listener, linear=(999, 999, 999), angular=(999, 999, 999))
        assert all(-1.0 <= v <= 1.0 for v in latest.values())

    def test_nan_input_gives_zero(self, executor, controller_node, publisher, listener):
        """A NaN command is replaced with 0.0 instead of reaching the thrusters."""
        latest = send_and_read(executor, publisher, listener, linear=(float('nan'), 0, 0))
        assert all(v == 0.0 for v in latest.values())

    def test_watchdog_zeroes_outputs(self, executor, controller_node, publisher, listener):
        """Commands drop to 0.0 once no Twist has arrived within the timeout."""
        latest = send_and_read(executor, publisher, listener, linear=(0, 0, 0.5))
        assert any(v != 0.0 for v in latest.values())
        spin_for(executor, controller_node.watchdog_timeout_s + 0.3)
        assert all(v == 0.0 for v in listener.latest().values())

    def test_last_command_wins(self, executor, controller_node, publisher, listener):
        """When several commands arrive together, the last one is applied."""
        for z in (0.2, 0.4, 0.6, 0.0):
            publisher.send(linear=(0, 0, z))
        spin_for(executor, 0.3)
        np.testing.assert_allclose(list(listener.latest().values()), 0.0, atol=1e-6)
