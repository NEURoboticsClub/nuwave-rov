"""Exercise the arm controller through its real ROS topics."""

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


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
    """Spin up the real ArmController node with the package config."""
    from arm_controller_pkg.arm_controller_node import ArmController
    node = ArmController()
    executor.add_node(node)
    yield node
    node.destroy_node()


class ArmMotorListener(Node):
    """Subscribe to every configured arm motor topic and record the commands."""

    def __init__(self, topics):
        super().__init__('test_arm_motor_listener')
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
        """Return the most recent command per motor, or None if nothing yet."""
        return {i: msgs[-1] if msgs else None for i, msgs in self.received.items()}


@pytest.fixture
def listener(executor, controller_node):
    """Listen on the topics the controller was configured with."""
    node = ArmMotorListener([m['topic'] for m in controller_node.motors])
    executor.add_node(node)
    yield node
    node.destroy_node()


class ArmCommandPublisher(Node):
    """Publish arm commands on the controller's input topic."""

    def __init__(self):
        super().__init__('test_arm_command_publisher')
        self.pub = self.create_publisher(Float32MultiArray, 'arm_commands', 10)

    def send(self, values):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        self.pub.publish(msg)


@pytest.fixture
def publisher(executor):
    """Provide an arm command publisher on the executor."""
    node = ArmCommandPublisher()
    executor.add_node(node)
    yield node
    node.destroy_node()


def spin_for(executor, seconds, step=0.01):
    """Spin the executor for a fixed duration to let messages flow."""
    end = time.time() + seconds
    while time.time() < end:
        executor.spin_once(timeout_sec=step)


def send_and_read(executor, publisher, listener, values):
    """Send one arm command, let the controller publish, and return the latest commands."""
    publisher.send(values)
    spin_for(executor, 0.3)
    latest = listener.latest()
    assert all(v is not None for v in latest.values()), f'Missing commands: {latest}'
    return latest


class TestNodeStartup:

    def test_one_publisher_per_configured_motor(self, controller_node):
        """Create one publisher for each arm motor in the config."""
        assert len(controller_node.motors) == 6
        assert all('pub' in m for m in controller_node.motors)

    def test_publishes_zero_without_input(self, executor, controller_node, listener):
        """Publish 0.0 on every motor before any arm command arrives."""
        spin_for(executor, 0.3)
        assert all(v == 0.0 for v in listener.latest().values())


class TestCommandMapping:

    def test_each_element_drives_matching_motor(
            self, executor, controller_node, publisher, listener):
        """Element i of the command drives the motor with id i + 1."""
        values = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        latest = send_and_read(executor, publisher, listener, values)
        for i, mot in enumerate(controller_node.motors):
            expected = values[mot['id'] - 1]
            assert latest[i] == pytest.approx(expected), f'Motor {mot["id"]}: {latest[i]}'

    def test_short_command_zeroes_missing_motors(
            self, executor, controller_node, publisher, listener):
        """Motors without a matching element in a short command are held at 0.0."""
        latest = send_and_read(executor, publisher, listener, [0.5, 0.5])
        for i, mot in enumerate(controller_node.motors):
            expected = 0.5 if mot['id'] <= 2 else 0.0
            assert latest[i] == pytest.approx(expected), f'Motor {mot["id"]}: {latest[i]}'

    def test_nan_element_gives_zero(self, executor, controller_node, publisher, listener):
        """A non-finite element is replaced with 0.0 for that motor only."""
        values = [float('nan'), 0.5, 0.5, 0.5, 0.5, 0.5]
        latest = send_and_read(executor, publisher, listener, values)
        for i, mot in enumerate(controller_node.motors):
            expected = 0.0 if mot['id'] == 1 else 0.5
            assert latest[i] == pytest.approx(expected), f'Motor {mot["id"]}: {latest[i]}'


class TestSafety:

    def test_watchdog_zeroes_outputs(self, executor, controller_node, publisher, listener):
        """Commands drop to 0.0 once no arm command has arrived within the timeout."""
        latest = send_and_read(executor, publisher, listener, [0.5] * 6)
        assert all(v == pytest.approx(0.5) for v in latest.values())
        spin_for(executor, controller_node.watchdog_timeout_s + 0.3)
        assert all(v == 0.0 for v in listener.latest().values())

    def test_last_command_wins(self, executor, controller_node, publisher, listener):
        """When several commands arrive together, the last one is applied."""
        for v in (0.2, 0.4, 0.0):
            publisher.send([v] * 6)
        spin_for(executor, 0.3)
        assert all(v == 0.0 for v in listener.latest().values())
