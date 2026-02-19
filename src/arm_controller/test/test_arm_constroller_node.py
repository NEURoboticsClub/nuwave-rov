import time
import pytest
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


@pytest.fixture(scope="module")
def ros_context():
    """Init/shutdown rclpy once for the whole test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def executor(ros_context):
    """Fresh executor per test."""
    exc = SingleThreadedExecutor()
    yield exc
    exc.shutdown()


@pytest.fixture
def controller_node(executor):
    """
    Spin up the real ThrusterController node.
    Import here so rclpy.init() has already run.
    """
    # Adjust this import path to match your package layout
    from arm_controller.arm_controller.arm_controller_node import ArmController
    node = ArmController()
    executor.add_node(node)
    yield node
    node.destroy_node()


class ArmMotorListener(Node):
    """
    Helper node that subscribes to all arm motor topics and collects messages.
    """
    def __init__(self, n_arm_motors: int = 8):
        super().__init__('test_arm_motor_listener')
        self.received = {}  # {channel_index: [list of Float32 values]}
        self.subs = []

        for i in range(n_arm_motors):
            topic = f'arm/arm_motor_{i}'
            self.received[i] = []
            sub = self.create_subscription(
                Float32, topic,
                self._make_cb(i),
                10
            )
            self.subs.append(sub)

    def _make_cb(self, index):
        def cb(msg: Float32):
            self.received[index].append(msg.data)
        return cb

    def clear(self):
        for k in self.received:
            self.received[k] = []

    def latest(self) -> dict:
        """Return the most recent value per channel, or None if nothing yet."""
        return {
            i: msgs[-1] if msgs else None
            for i, msgs in self.received.items()
        }


@pytest.fixture
def listener(executor):
    node = ArmMotorListener(n_arm_motors=8)
    executor.add_node(node)
    yield node
    node.destroy_node()


class TwistPublisher(Node):
    """Helper node that publishes Twist messages on the status topic."""
    def __init__(self):
        super().__init__('test_twist_publisher')
        self.pub = self.create_publisher(Twist, 'status_topic', 10)

    def send(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = [float(v) for v in linear]
        msg.angular.x, msg.angular.y, msg.angular.z = [float(v) for v in angular]
        self.pub.publish(msg)


@pytest.fixture
def publisher(executor):
    node = TwistPublisher()
    executor.add_node(node)
    yield node
    node.destroy_node()


# Helpers
def spin_for(executor, seconds: float, step: float = 0.01):
    """Spin the executor for a fixed duration to let messages flow."""
    end = time.time() + seconds
    while time.time() < end:
        executor.spin_once(timeout_sec=step)

# Tests
class TestNodeStartup:
    def test_node_creates_publishers(self, controller_node):
        """Controller should create one publisher per arm motor."""
        assert len(controller_node.arm_motor_pubs) == controller_node.n_arm_motors

    def test_initial_pwm_is_neutral(self, controller_node):
        """Before any message, all commands should be neutral."""
        expected = controller_node.neutral_us
        np.testing.assert_allclose(controller_node.pwm_commands, expected)


class TestZeroInput:
    def test_zero_twist_gives_neutral_pwm(self, executor, controller_node, publisher, listener):
        """Publishing a zero Twist should result in neutral PWM on all channels."""
        publisher.send(linear=(0, 0, 0), angular=(0, 0, 0))
        spin_for(executor, 0.5)  # let timer fire a few times
    
        latest = listener.latest()
        for i, val in latest.items():
            assert val is not None, f"No message received on arm motor {i}"
            assert abs(val - controller_node.neutral_us) < 1.0, \
                f"Arm motor {i}: expected ~{controller_node.neutral_us}, got {val}"


class TestDirectionalResponse:
    def test_pure_heave_activates_vertical_only(self, executor, controller_node, publisher, listener):
        """A pure Z force should only change vertical thruster PWM values."""
        publisher.send(linear=(0, 0, 20.0), angular=(0, 0, 0))
        spin_for(executor, 0.5)

        latest = listener.latest()
        neutral = controller_node.neutral_us

        # Lateral thrusters (0-3) should stay at neutral
        for i in range(4):
            assert latest[i] is not None
            assert abs(latest[i] - neutral) < 1.0, \
                f"Lateral thruster {i} moved during pure heave: {latest[i]}"

        # Vertical thrusters (4-7) should be above neutral (pushing up)
        for i in range(4, 8):
            assert latest[i] is not None
            assert latest[i] > neutral + 1.0, \
                f"Vertical thruster {i} didn't respond to heave: {latest[i]}"

    def test_pure_surge_activates_lateral_only(self, executor, controller_node, publisher, listener):
        """A pure Y force should only change lateral thruster PWM values."""
        publisher.send(linear=(0, 20.0, 0), angular=(0, 0, 0))
        spin_for(executor, 0.5)

        latest = listener.latest()
        neutral = controller_node.neutral_us

        # Vertical thrusters (4-7) should stay at neutral
        for i in range(4, 8):
            assert latest[i] is not None
            assert abs(latest[i] - neutral) < 1.0, \
                f"Vertical thruster {i} moved during pure surge: {latest[i]}"

    def test_opposite_commands_give_opposite_pwm(self, executor, controller_node, publisher, listener):
        """Forward and backward commands should produce symmetric PWM offsets."""
        # Forward
        publisher.send(linear=(0, 10.0, 0))
        spin_for(executor, 0.3)
        fwd = listener.latest()

        listener.clear()

        # Backward
        publisher.send(linear=(0, -10.0, 0))
        spin_for(executor, 0.3)
        rev = listener.latest()

        neutral = controller_node.neutral_us
        for i in range(4):  # lateral thrusters
            if fwd[i] is not None and rev[i] is not None:
                fwd_offset = fwd[i] - neutral
                rev_offset = rev[i] - neutral
                assert abs(fwd_offset + rev_offset) < 5.0, \
                    f"Thruster {i} not symmetric: fwd={fwd_offset}, rev={rev_offset}"


class TestPWMBounds:
    def test_large_input_stays_in_bounds(self, executor, controller_node, publisher, listener):
        """Even with extreme input, PWM should never exceed [min_us, max_us]."""
        publisher.send(linear=(999, 999, 999), angular=(999, 999, 999))
        spin_for(executor, 0.5)

        latest = listener.latest()
        for i, val in latest.items():
            assert val is not None, f"No message on thruster {i}"
            assert val >= controller_node.min_us, \
                f"Thruster {i} below min: {val}"
            assert val <= controller_node.max_us, \
                f"Thruster {i} above max: {val}"


class TestMessageFlow:
    def test_timer_publishes_without_input(self, executor, controller_node, listener):
        """Timer should publish neutral even with no Twist input."""
        spin_for(executor, 0.5)

        latest = listener.latest()
        received_count = sum(1 for v in latest.values() if v is not None)
        assert received_count == controller_node.n_arm_motors, \
            f"Only {received_count}/{controller_node.n_arm_motors} channels received"

    def test_multiple_messages_last_one_wins(self, executor, controller_node, publisher, listener):
        """Rapidly publishing should result in the last command being active."""
        for val in [5.0, 10.0, 20.0, 0.0]:
            publisher.send(linear=(0, val, 0))
        spin_for(executor, 0.5)

        # After sending 0, should be back to neutral
        latest = listener.latest()
        neutral = controller_node.neutral_us
        for i in range(controller_node.n_arm_motors):
            if latest[i] is not None:
                assert abs(latest[i] - neutral) < 5.0, \
                    f"Arm motor {i} not back to neutral: {latest[i]}"
