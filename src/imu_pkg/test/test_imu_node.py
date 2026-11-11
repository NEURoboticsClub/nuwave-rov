"""
Unit tests for the IMUPublisher node.

ROS (rclpy, sensor_msgs) and the I2C driver are replaced with light fakes so the tests run without
ROS or hardware. Only the logic in imu_node.py is exercised.
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

PKG_DIR = Path(__file__).resolve().parents[1]
SQRT_HALF = 2 ** -0.5

# Default mount quaternion in the node is [x, y, z, w] = [0, 0, -sqrt(.5), sqrt(.5)],
# i.e. a -90 degree rotation about z. In (w, x, y, z) order the node stores it as:
DEFAULT_QM = [SQRT_HALF, 0.0, 0.0, -SQRT_HALF]
IDENTITY_MOUNT = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]


class StopLoop(BaseException):
    """Raised by a fake time.sleep to break the node's infinite driver-retry loop."""


class FakeNode:
    """Minimal stand-in for rclpy.node.Node that records what the node asks of ROS."""

    parameter_overrides = {}
    instances = []

    def __init__(self, name):
        self.node_name = name
        self.logger = Mock()
        self.publisher = Mock()
        self.timer = Mock()
        self.stamp = object()
        self.created_publishers = []
        self.created_timers = []
        self.destroyed = False
        FakeNode.instances.append(self)

    def declare_parameter(self, name, default):
        value = self.parameter_overrides.get(name, default)
        return SimpleNamespace(value=value)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        stamp = self.stamp
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp))

    def create_publisher(self, msg_type, topic, depth):
        self.created_publishers.append((msg_type, topic, depth))
        return self.publisher

    def create_timer(self, period, callback):
        self.created_timers.append((period, callback))
        return self.timer

    def destroy_node(self):
        self.destroyed = True


class FakeImu:
    """Stand-in for sensor_msgs.msg.Imu with the fields the node fills in."""

    def __init__(self):
        self.header = SimpleNamespace(frame_id='', stamp=None)
        self.orientation = SimpleNamespace(w=0.0, x=0.0, y=0.0, z=0.0)
        self.angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.linear_acceleration = SimpleNamespace(x=0.0, y=0.0, z=0.0)


def make_reading(accel=(1.0, 2.0, 3.0), gyro=(4.0, 5.0, 6.0), quat=(1.0, 0.0, 0.0, 0.0)):
    """Build a dict shaped like ImuDriver.read_data() output."""
    return {
        'linear_acceleration': {'accel_x': accel[0], 'accel_y': accel[1], 'accel_z': accel[2]},
        'angular_velocity': {'gyro_x': gyro[0], 'gyro_y': gyro[1], 'gyro_z': gyro[2]},
        'game_quaternion': {'real': quat[0], 'i': quat[1], 'j': quat[2], 'k': quat[3]},
    }


@pytest.fixture
def env(monkeypatch):
    """Load imu_node.py against fake ROS modules and a mocked ImuDriver."""
    FakeNode.parameter_overrides = {}
    FakeNode.instances = []

    def install(name, **attrs):
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    rclpy = install('rclpy', init=Mock(), spin=Mock(), shutdown=Mock())
    install('rclpy.node', Node=FakeNode)
    install('sensor_msgs')
    install('sensor_msgs.msg', Imu=FakeImu)

    driver_cls = Mock(name='ImuDriver')
    driver_cls.return_value.read_data.return_value = make_reading()
    install('imu_pkg.imu_driver', ImuDriver=driver_cls)

    monkeypatch.syspath_prepend(str(PKG_DIR))
    spec = importlib.util.spec_from_file_location(
        '_imu_node_under_test', PKG_DIR / 'imu_pkg' / 'imu_node.py')
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)

    # Never really sleep; abort the retry loop if a test lets it spin too long.
    sleep = Mock(side_effect=[None] * 5 + [StopLoop()])
    monkeypatch.setattr(module.time, 'sleep', sleep)

    return SimpleNamespace(module=module, rclpy=rclpy, driver_cls=driver_cls,
                           driver=driver_cls.return_value, sleep=sleep)


@pytest.fixture
def node(env):
    """A fully constructed IMUPublisher with default parameters."""
    return env.module.IMUPublisher()


def published(node):
    """Return the Imu messages passed to publisher.publish()."""
    return [call.args[0] for call in node.publisher.publish.call_args_list]


# --------------------------------------------------------------------------------------
# _quat_mul
# --------------------------------------------------------------------------------------

@pytest.fixture
def quat_mul(env):
    return env.module.IMUPublisher._quat_mul


@pytest.mark.parametrize('q', [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0.5, -0.5, 0.5, -0.5],
    [0.1, 0.2, -0.3, 0.4],
])
def test_quat_mul_identity_is_neutral_on_both_sides(quat_mul, q):
    identity = [1, 0, 0, 0]
    assert quat_mul(identity, q) == pytest.approx(q)
    assert quat_mul(q, identity) == pytest.approx(q)


@pytest.mark.parametrize('a, b, expected', [
    ([0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]),    # i * j = k
    ([0, 0, 1, 0], [0, 0, 0, 1], [0, 1, 0, 0]),    # j * k = i
    ([0, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0]),    # k * i = j
    ([0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, -1]),   # j * i = -k
    ([0, 1, 0, 0], [0, 1, 0, 0], [-1, 0, 0, 0]),   # i * i = -1
    ([0, 0, 1, 0], [0, 0, 1, 0], [-1, 0, 0, 0]),   # j * j = -1
    ([0, 0, 0, 1], [0, 0, 0, 1], [-1, 0, 0, 0]),   # k * k = -1
])
def test_quat_mul_basis_products(quat_mul, a, b, expected):
    assert quat_mul(a, b) == pytest.approx(expected)


def test_quat_mul_is_not_commutative(quat_mul):
    a, b = [0, 1, 0, 0], [0, 0, 1, 0]
    assert not np.allclose(quat_mul(a, b), quat_mul(b, a))


def test_quat_mul_general_product_matches_hand_calculation(quat_mul):
    # (1 + 2i + 3j + 4k)(5 + 6i + 7j + 8k) = -60 + 12i + 30j + 24k
    assert quat_mul([1, 2, 3, 4], [5, 6, 7, 8]) == pytest.approx([-60, 12, 30, 24])


def test_quat_mul_is_associative(quat_mul):
    a, b, c = [0.1, 0.2, 0.3, 0.4], [-0.5, 0.1, 0.0, 0.7], [0.3, -0.2, 0.9, 0.1]
    assert quat_mul(quat_mul(a, b), c) == pytest.approx(quat_mul(a, quat_mul(b, c)))


def test_quat_mul_by_conjugate_gives_squared_norm(quat_mul):
    q = np.array([0.1, 0.2, -0.3, 0.4])
    conj = np.array([q[0], -q[1], -q[2], -q[3]])
    assert quat_mul(q, conj) == pytest.approx([np.dot(q, q), 0, 0, 0])


def test_quat_mul_preserves_unit_norm(quat_mul):
    rng = np.random.default_rng(0)
    a, b = rng.normal(size=4), rng.normal(size=4)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    assert np.linalg.norm(quat_mul(a, b)) == pytest.approx(1.0)


def test_quat_mul_zero_quaternion_gives_zero(quat_mul):
    assert quat_mul([0, 0, 0, 0], [1, 2, 3, 4]) == pytest.approx([0, 0, 0, 0])


def test_quat_mul_returns_length_four_float_array(quat_mul):
    result = quat_mul((1, 0, 0, 0), np.array([0.0, 1.0, 0.0, 0.0]))  # tuple and ndarray inputs
    assert isinstance(result, np.ndarray)
    assert result.shape == (4,)


@pytest.mark.parametrize('bad', [[], [1, 0, 0], [1, 0, 0, 0, 0]])
def test_quat_mul_rejects_wrong_length_input(quat_mul, bad):
    with pytest.raises(ValueError):
        quat_mul(bad, [1, 0, 0, 0])
    with pytest.raises(ValueError):
        quat_mul([1, 0, 0, 0], bad)


# --------------------------------------------------------------------------------------
# _rotate_vec
# --------------------------------------------------------------------------------------

@pytest.fixture
def rotate_vec(env):
    return env.module.IMUPublisher._rotate_vec


def axis_angle(axis, degrees):
    """Unit quaternion (w, x, y, z) for a rotation of the given degrees about the axis."""
    half = np.radians(degrees) / 2
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    return np.concatenate([[np.cos(half)], np.sin(half) * axis])


def test_rotate_vec_identity_leaves_vector_unchanged(rotate_vec):
    assert rotate_vec([1, 0, 0, 0], [1.5, -2.0, 3.0]) == pytest.approx([1.5, -2.0, 3.0])


@pytest.mark.parametrize('axis, degrees, vec, expected', [
    ([0, 0, 1], 90, [1, 0, 0], [0, 1, 0]),
    ([0, 0, 1], -90, [1, 0, 0], [0, -1, 0]),
    ([0, 0, 1], 180, [1, 2, 3], [-1, -2, 3]),
    ([1, 0, 0], 90, [0, 1, 0], [0, 0, 1]),
    ([1, 0, 0], 90, [0, 0, 1], [0, -1, 0]),
    ([0, 1, 0], 90, [0, 0, 1], [1, 0, 0]),
    ([0, 1, 0], 90, [1, 0, 0], [0, 0, -1]),
    ([0, 0, 1], 360, [1, 2, 3], [1, 2, 3]),
])
def test_rotate_vec_known_rotations(rotate_vec, axis, degrees, vec, expected):
    assert rotate_vec(axis_angle(axis, degrees), vec) == pytest.approx(expected, abs=1e-12)


def test_rotate_vec_vector_on_rotation_axis_is_unchanged(rotate_vec):
    q = axis_angle([1, 1, 1], 73)
    assert rotate_vec(q, [2, 2, 2]) == pytest.approx([2, 2, 2])


def test_rotate_vec_zero_vector_stays_zero(rotate_vec):
    assert rotate_vec(axis_angle([1, 2, 3], 40), [0, 0, 0]) == pytest.approx([0, 0, 0])


def test_rotate_vec_preserves_length(rotate_vec):
    rng = np.random.default_rng(1)
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    v = rng.normal(size=3)
    assert np.linalg.norm(rotate_vec(q, v)) == pytest.approx(np.linalg.norm(v))


def test_rotate_vec_conjugate_undoes_rotation(rotate_vec):
    q = axis_angle([1, -2, 0.5], 55)
    conj = [q[0], -q[1], -q[2], -q[3]]
    v = [0.3, -1.2, 2.2]
    assert rotate_vec(conj, rotate_vec(q, v)) == pytest.approx(v)


def test_rotate_vec_composition_matches_quaternion_product(env, rotate_vec):
    quat_mul = env.module.IMUPublisher._quat_mul
    q1, q2 = axis_angle([0, 0, 1], 30), axis_angle([1, 0, 0], 60)
    v = [1.0, 2.0, 3.0]
    assert rotate_vec(quat_mul(q1, q2), v) == pytest.approx(rotate_vec(q1, rotate_vec(q2, v)))


def test_rotate_vec_returns_length_three_array(rotate_vec):
    result = rotate_vec((1, 0, 0, 0), (1, 2, 3))  # tuple inputs
    assert isinstance(result, np.ndarray)
    assert result.shape == (3,)


@pytest.mark.parametrize('bad_q, bad_v', [
    ([1, 0, 0], [1, 2, 3]),
    ([1, 0, 0, 0], [1, 2]),
    ([1, 0, 0, 0], [1, 2, 3, 4]),
])
def test_rotate_vec_rejects_wrong_length_input(rotate_vec, bad_q, bad_v):
    with pytest.raises(ValueError):
        rotate_vec(bad_q, bad_v)


# --------------------------------------------------------------------------------------
# __init__
# --------------------------------------------------------------------------------------

def test_init_uses_default_parameters_and_wires_ros(env, node):
    assert node.node_name == 'imu'
    env.driver_cls.assert_called_once_with(7, 100)
    assert node.imu is env.driver
    assert node.created_publishers == [(FakeImu, 'imu', 10)]
    assert node.publisher_ is node.publisher
    assert len(node.created_timers) == 1
    period, callback = node.created_timers[0]
    assert period == pytest.approx(0.01)
    assert callback == node.timer_callback
    node.logger.info.assert_called_once_with('IMU initialized')
    node.logger.error.assert_not_called()


def test_init_converts_mount_quaternion_to_wxyz_and_builds_conjugate(node):
    assert node.q_mount == [0.0, 0.0, -0.7071068, 0.7071068]
    assert node._qm == pytest.approx(DEFAULT_QM, abs=1e-6)
    assert node._qm_inv == pytest.approx([DEFAULT_QM[0], 0, 0, SQRT_HALF], abs=1e-6)


def test_init_honours_parameter_overrides(env):
    FakeNode.parameter_overrides = {
        'addr': 1, 'sampling_rate': 50, 'q_mount': [0.1, 0.2, 0.3, 0.9]}
    node = env.module.IMUPublisher()
    env.driver_cls.assert_called_once_with(1, 50)
    assert node.created_timers[0][0] == pytest.approx(0.02)
    assert node._qm == pytest.approx([0.9, 0.1, 0.2, 0.3])
    assert node._qm_inv == pytest.approx([0.9, -0.1, -0.2, -0.3])


def test_init_retries_driver_until_it_succeeds(env):
    env.driver_cls.side_effect = [OSError('bus busy'), RuntimeError('no ack'), env.driver]
    node = env.module.IMUPublisher()
    assert env.driver_cls.call_count == 3
    assert env.sleep.call_args_list == [((1.0,),), ((1.0,),)]
    assert node.imu is env.driver
    assert node.logger.error.call_count == 2
    assert 'bus busy' in node.logger.error.call_args_list[0].args[0]
    assert 'no ack' in node.logger.error.call_args_list[1].args[0]
    # Publisher and timer are only created after the driver comes up.
    assert len(node.created_publishers) == 1
    assert len(node.created_timers) == 1
    node.logger.info.assert_called_once_with('IMU initialized')


def test_init_keeps_retrying_while_driver_keeps_failing(env):
    env.driver_cls.side_effect = OSError('dead sensor')
    with pytest.raises(StopLoop):
        env.module.IMUPublisher()
    assert env.driver_cls.call_count == env.sleep.call_count


# --------------------------------------------------------------------------------------
# timer_callback
# --------------------------------------------------------------------------------------

def use_identity_mount(env):
    FakeNode.parameter_overrides = {'q_mount': IDENTITY_MOUNT}


def test_timer_callback_publishes_one_message_with_header(env, node):
    node.timer_callback()
    msgs = published(node)
    assert len(msgs) == 1
    assert isinstance(msgs[0], FakeImu)
    assert msgs[0].header.frame_id == 'imu_link'
    assert msgs[0].header.stamp is node.stamp
    env.driver.read_data.assert_called_once_with()


def test_timer_callback_passes_data_through_with_identity_mount(env):
    use_identity_mount(env)
    node = env.module.IMUPublisher()
    env.driver.read_data.return_value = make_reading(quat=(0.5, 0.5, 0.5, 0.5))
    node.timer_callback()
    msg = published(node)[0]
    o = msg.orientation
    assert [o.w, o.x, o.y, o.z] == pytest.approx([0.5, 0.5, 0.5, 0.5])
    av, la = msg.angular_velocity, msg.linear_acceleration
    assert [av.x, av.y, av.z] == pytest.approx([4, 5, 6])
    assert [la.x, la.y, la.z] == pytest.approx([1, 2, 3])


def test_timer_callback_applies_default_mount_to_vectors_and_orientation(env, node):
    env.driver.read_data.return_value = make_reading(
        accel=(1, 2, 3), gyro=(4, 5, 6), quat=(1, 0, 0, 0))
    node.timer_callback()
    msg = published(node)[0]
    # Default mount is -90 deg about z: (x, y, z) -> (y, -x, z)
    av, la, o = msg.angular_velocity, msg.linear_acceleration, msg.orientation
    assert [av.x, av.y, av.z] == pytest.approx([5, -4, 6], abs=1e-6)
    assert [la.x, la.y, la.z] == pytest.approx([2, -1, 3], abs=1e-6)
    # q_body = q_chip * q_mount^-1 = identity * (+90 deg about z)
    assert [o.w, o.x, o.y, o.z] == pytest.approx([SQRT_HALF, 0, 0, SQRT_HALF], abs=1e-6)


def test_timer_callback_mounted_sensor_yaw_cancels_in_orientation(env, node):
    # Chip reports the mount rotation itself -> body frame is aligned with the world.
    env.driver.read_data.return_value = make_reading(quat=tuple(DEFAULT_QM))
    node.timer_callback()
    o = published(node)[0].orientation
    assert [o.w, o.x, o.y, o.z] == pytest.approx([1, 0, 0, 0], abs=1e-6)


def test_timer_callback_publishes_python_floats(env, node):
    node.timer_callback()
    msg = published(node)[0]
    for group in (msg.orientation, msg.angular_velocity, msg.linear_acceleration):
        for value in vars(group).values():
            assert type(value) is float


def test_timer_callback_all_zero_vectors_publish_zeros(env, node):
    env.driver.read_data.return_value = make_reading(accel=(0, 0, 0), gyro=(0, 0, 0))
    node.timer_callback()
    msg = published(node)[0]
    assert vars(msg.angular_velocity) == {'x': 0.0, 'y': 0.0, 'z': 0.0}
    assert vars(msg.linear_acceleration) == {'x': 0.0, 'y': 0.0, 'z': 0.0}


def test_timer_callback_reads_fresh_data_every_call(env):
    use_identity_mount(env)
    node = env.module.IMUPublisher()
    env.driver.read_data.side_effect = [
        make_reading(accel=(1, 0, 0)), make_reading(accel=(9, 0, 0))]
    node.timer_callback()
    node.timer_callback()
    assert [m.linear_acceleration.x for m in published(node)] == [1.0, 9.0]


def test_timer_callback_does_nothing_without_a_driver(env, node):
    node.imu = None
    node.timer_callback()
    env.driver.read_data.assert_not_called()
    node.publisher.publish.assert_not_called()
    node.logger.error.assert_not_called()


def test_timer_callback_skips_cycle_when_read_raises_then_recovers(env, node):
    env.driver.read_data.side_effect = [OSError('i2c timeout'), make_reading()]
    node.timer_callback()
    node.publisher.publish.assert_not_called()
    node.logger.error.assert_called_once()
    assert 'i2c timeout' in node.logger.error.call_args.args[0]
    node.timer_callback()
    assert len(published(node)) == 1


@pytest.mark.parametrize('missing', ['linear_acceleration', 'angular_velocity', 'game_quaternion'])
def test_timer_callback_skips_cycle_when_a_section_is_missing(env, node, missing):
    reading = make_reading()
    del reading[missing]
    env.driver.read_data.return_value = reading
    node.timer_callback()
    node.publisher.publish.assert_not_called()
    node.logger.error.assert_called_once()


def test_timer_callback_skips_cycle_when_reading_is_empty_or_none(env, node):
    env.driver.read_data.side_effect = [{}, None]
    node.timer_callback()
    node.timer_callback()
    node.publisher.publish.assert_not_called()
    assert node.logger.error.call_count == 2


# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------

def test_main_initialises_spins_and_shuts_down(env):
    env.module.main(args=['--ros-args'])
    env.rclpy.init.assert_called_once_with(args=['--ros-args'])
    assert len(FakeNode.instances) == 1
    env.rclpy.spin.assert_called_once_with(FakeNode.instances[0])
    assert FakeNode.instances[0].destroyed
    env.rclpy.shutdown.assert_called_once_with()


def test_main_defaults_to_no_args(env):
    env.module.main()
    env.rclpy.init.assert_called_once_with(args=None)


def test_main_swallows_keyboard_interrupt_and_cleans_up(env):
    env.rclpy.spin.side_effect = KeyboardInterrupt
    env.module.main()
    assert FakeNode.instances[0].destroyed
    env.rclpy.shutdown.assert_called_once_with()


def test_main_cleans_up_then_propagates_other_errors(env):
    env.rclpy.spin.side_effect = RuntimeError('executor died')
    with pytest.raises(RuntimeError, match='executor died'):
        env.module.main()
    assert FakeNode.instances[0].destroyed
    env.rclpy.shutdown.assert_called_once_with()


def test_main_cleanup_order_is_destroy_before_shutdown(env):
    order = []
    env.rclpy.shutdown.side_effect = lambda: order.append('shutdown')
    original_destroy = FakeNode.destroy_node
    FakeNode.destroy_node = lambda self: (order.append('destroy'), original_destroy(self))
    try:
        env.module.main()
    finally:
        FakeNode.destroy_node = original_destroy
    assert order == ['destroy', 'shutdown']
