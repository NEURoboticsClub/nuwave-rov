from unittest.mock import MagicMock

from geometry_msgs.msg import Quaternion, Twist
import numpy as np
import pytest
from sensor_msgs.msg import Imu
from stabilization_pkg import stabilization_functions
from std_msgs.msg import Empty, Float32


def make_state(**overrides):
    """Build a state dict matching the one created by StabilizationNode."""
    state = {
        'desired_orientation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        'desired_depth': None,
        'last_imu_orientation': None,
        'last_imu_msg': None,
        'imu_sample_time': None,
        'current_depth': None,
        'depth_rate': 0.0,
        'depth_sample_time': None,
    }
    state.update(overrides)
    return state


def make_imu(w=1.0, x=0.0, y=0.0, z=0.0, gyro=(0.0, 0.0, 0.0)):
    msg = Imu()
    msg.orientation = Quaternion(w=float(w), x=float(x), y=float(y), z=float(z))
    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])
    return msg


def axis_angle_quat(axis, angle):
    """Return a Quaternion msg rotating by angle (rad) about a unit axis."""
    s = np.sin(angle / 2.0)
    return Quaternion(
        w=float(np.cos(angle / 2.0)),
        x=float(axis[0] * s),
        y=float(axis[1] * s),
        z=float(axis[2] * s),
    )


def quat_values(q):
    return np.array([q.w, q.x, q.y, q.z])


def twist_angular(twist):
    return np.array([twist.angular.x, twist.angular.y, twist.angular.z])


# _quat_mul

def test_quat_mul_identity():
    q = np.array([0.5, 0.5, 0.5, 0.5])
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(stabilization_functions._quat_mul(identity, q), q)
    assert np.allclose(stabilization_functions._quat_mul(q, identity), q)


def test_quat_mul_known_product():
    # 90 deg about x, then 90 deg about y (Hamilton product q1 * q2)
    h = np.sqrt(0.5)
    q1 = np.array([h, h, 0.0, 0.0])
    q2 = np.array([h, 0.0, h, 0.0])
    result = stabilization_functions._quat_mul(q1, q2)
    assert np.allclose(result, [0.5, 0.5, 0.5, 0.5])


def test_quat_mul_with_conjugate_is_identity():
    q = quat_values(axis_angle_quat([0.0, 0.6, 0.8], 1.2))
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    result = stabilization_functions._quat_mul(q, q_conj)
    assert np.allclose(result, [1.0, 0.0, 0.0, 0.0])


# set_desired_orientation

def test_set_desired_orientation_normalizes_input():
    state = make_state()
    logger = MagicMock()
    q = Quaternion(w=2.0, x=2.0, y=0.0, z=0.0)

    assert stabilization_functions.set_desired_orientation(q, state, logger)

    h = np.sqrt(0.5)
    assert np.allclose(quat_values(state['desired_orientation']), [h, h, 0.0, 0.0])
    logger.info.assert_called_once()


@pytest.mark.parametrize('q', [
    Quaternion(w=float('nan'), x=0.0, y=0.0, z=0.0),
    Quaternion(w=1.0, x=float('inf'), y=0.0, z=0.0),
    Quaternion(w=0.0, x=0.0, y=0.0, z=0.0),
])
def test_set_desired_orientation_rejects_invalid_input(q):
    original = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    state = make_state(desired_orientation=original)
    logger = MagicMock()

    assert not stabilization_functions.set_desired_orientation(q, state, logger)

    assert state['desired_orientation'] is original
    logger.warn.assert_called_once()


# _imu_to_twist_control

PD_PARAMS = {'p': 1.0, 'd': 0.0, 'max_stabilization_influence': 10.0}


def test_imu_to_twist_control_at_setpoint_is_zero():
    twist = stabilization_functions._imu_to_twist_control(
        make_imu(), Quaternion(w=1.0), PD_PARAMS, MagicMock())
    assert isinstance(twist, Twist)
    assert np.allclose(twist_angular(twist), 0.0)


def test_imu_to_twist_control_proportional_term():
    angle = 0.2
    desired = axis_angle_quat([1.0, 0.0, 0.0], angle)
    params = {'p': 2.0, 'd': 0.0, 'max_stabilization_influence': 10.0}

    twist = stabilization_functions._imu_to_twist_control(
        make_imu(), desired, params, MagicMock())

    # Error vector is 2 * sin(angle / 2) about the error axis
    expected_x = 2.0 * 2.0 * np.sin(angle / 2.0)
    assert np.allclose(twist_angular(twist), [expected_x, 0.0, 0.0])


def test_imu_to_twist_control_derivative_term():
    params = {'p': 1.0, 'd': 0.5, 'max_stabilization_influence': 10.0}
    imu = make_imu(gyro=(0.1, 0.2, 0.3))

    twist = stabilization_functions._imu_to_twist_control(
        imu, Quaternion(w=1.0), params, MagicMock())

    # Yaw is always zeroed
    assert np.allclose(twist_angular(twist), [-0.05, -0.1, 0.0])


def test_imu_to_twist_control_zeroes_yaw():
    desired = axis_angle_quat([0.0, 0.0, 1.0], 0.5)

    twist = stabilization_functions._imu_to_twist_control(
        make_imu(), desired, PD_PARAMS, MagicMock())

    assert np.allclose(twist_angular(twist), 0.0)


def test_imu_to_twist_control_clamps_to_max_influence():
    desired = axis_angle_quat([1.0, 0.0, 0.0], 1.0)
    params = {'p': 10.0, 'd': 0.0, 'max_stabilization_influence': 0.3}

    twist = stabilization_functions._imu_to_twist_control(
        make_imu(), desired, params, MagicMock())

    assert np.allclose(twist_angular(twist), [0.3, 0.0, 0.0])


def test_imu_to_twist_control_handles_quaternion_double_cover():
    desired = axis_angle_quat([0.0, 1.0, 0.0], 0.4)
    negated = Quaternion(w=-desired.w, x=-desired.x, y=-desired.y, z=-desired.z)

    twist = stabilization_functions._imu_to_twist_control(
        make_imu(), desired, PD_PARAMS, MagicMock())
    twist_negated = stabilization_functions._imu_to_twist_control(
        make_imu(), negated, PD_PARAMS, MagicMock())

    assert np.allclose(twist_angular(twist), twist_angular(twist_negated))
    assert twist.angular.y > 0.0


def test_imu_to_twist_control_normalizes_current_orientation():
    current = axis_angle_quat([1.0, 0.0, 0.0], 0.3)
    scaled = make_imu(w=3 * current.w, x=3 * current.x, y=3 * current.y, z=3 * current.z)
    unit = make_imu(w=current.w, x=current.x, y=current.y, z=current.z)

    twist_scaled = stabilization_functions._imu_to_twist_control(
        scaled, Quaternion(w=1.0), PD_PARAMS, MagicMock())
    twist_unit = stabilization_functions._imu_to_twist_control(
        unit, Quaternion(w=1.0), PD_PARAMS, MagicMock())

    assert np.allclose(twist_angular(twist_scaled), twist_angular(twist_unit))


@pytest.mark.parametrize('imu', [
    make_imu(x=float('nan')),
    make_imu(gyro=(0.0, float('inf'), 0.0)),
])
def test_imu_to_twist_control_rejects_non_finite_data(imu):
    logger = MagicMock()

    twist = stabilization_functions._imu_to_twist_control(
        imu, Quaternion(w=1.0), PD_PARAMS, logger)

    assert np.allclose(twist_angular(twist), 0.0)
    logger.warn.assert_called_once()


def test_imu_to_twist_control_zero_norm_orientation_is_zero():
    twist = stabilization_functions._imu_to_twist_control(
        make_imu(w=0.0), Quaternion(w=1.0), PD_PARAMS, MagicMock())
    assert np.allclose(twist_angular(twist), 0.0)


# _depth_to_twist_control

def test_depth_to_twist_control_pd_command():
    params = {'p': 0.5, 'd': 1.0, 'max_stabilization_influence': 1.0}
    command = stabilization_functions._depth_to_twist_control(5.0, 0.2, 4.0, params)
    assert command == pytest.approx(0.5 * 1.0 + 1.0 * 0.2)


def test_depth_to_twist_control_defaults():
    command = stabilization_functions._depth_to_twist_control(4.5, 0.3, 4.0, {})
    # Defaults: p=1, d=0
    assert command == pytest.approx(0.5)


@pytest.mark.parametrize('current_depth, expected', [(10.0, 0.4), (-10.0, -0.4)])
def test_depth_to_twist_control_clamps(current_depth, expected):
    params = {'p': 1.0, 'd': 0.0, 'max_stabilization_influence': 0.4}
    command = stabilization_functions._depth_to_twist_control(
        current_depth, 0.0, 0.0, params)
    assert command == pytest.approx(expected)


def test_depth_to_twist_control_negative_max_influence_is_zero():
    params = {'p': 1.0, 'd': 0.0, 'max_stabilization_influence': -1.0}
    command = stabilization_functions._depth_to_twist_control(5.0, 0.0, 0.0, params)
    assert command == 0.0


# depth_callback

def test_depth_callback_first_sample():
    state = make_state()

    stabilization_functions.depth_callback(
        Float32(data=5.0), state, 10.0, 0.5, 0.5, MagicMock())

    assert state['current_depth'] == pytest.approx(5.0)
    assert state['depth_rate'] == 0.0
    assert state['depth_sample_time'] == 10.0


def test_depth_callback_filters_depth_rate():
    state = make_state(current_depth=4.0, depth_rate=0.4, depth_sample_time=10.0)

    stabilization_functions.depth_callback(
        Float32(data=5.0), state, 10.5, 1.0, 0.5, MagicMock())

    # raw rate = 1.0 m / 0.5 s = 2.0; filtered = 0.5 * 2.0 + 0.5 * 0.4
    assert state['depth_rate'] == pytest.approx(1.2)
    assert state['current_depth'] == pytest.approx(5.0)
    assert state['depth_sample_time'] == 10.5


def test_depth_callback_clips_filter_alpha():
    state = make_state(current_depth=4.0, depth_rate=0.4, depth_sample_time=10.0)

    stabilization_functions.depth_callback(
        Float32(data=5.0), state, 10.5, 1.0, 5.0, MagicMock())

    # alpha clipped to 1.0 -> raw rate only
    assert state['depth_rate'] == pytest.approx(2.0)


@pytest.mark.parametrize('now', [12.0, 10.0, 9.0])
def test_depth_callback_resets_rate_on_stale_or_bad_timing(now):
    state = make_state(current_depth=4.0, depth_rate=0.4, depth_sample_time=10.0)

    stabilization_functions.depth_callback(
        Float32(data=5.0), state, now, 1.0, 0.5, MagicMock())

    assert state['depth_rate'] == 0.0
    assert state['current_depth'] == pytest.approx(5.0)
    assert state['depth_sample_time'] == now


def test_depth_callback_rejects_non_finite_depth():
    state = make_state(current_depth=4.0, depth_rate=0.4, depth_sample_time=10.0)
    logger = MagicMock()

    stabilization_functions.depth_callback(
        Float32(data=float('nan')), state, 10.5, 1.0, 0.5, logger)

    assert state['current_depth'] == 4.0
    assert state['depth_rate'] == 0.4
    assert state['depth_sample_time'] == 10.0
    logger.warn.assert_called_once()


# imu_callback

def test_imu_callback_caches_sample():
    state = make_state()
    msg = make_imu(w=0.0, x=1.0)

    stabilization_functions.imu_callback(msg, state, 10.0)

    assert state['last_imu_msg'] is msg
    assert state['last_imu_orientation'] is msg.orientation
    assert state['imu_sample_time'] == 10.0


# capture_callback

def test_capture_callback_captures_orientation_and_depth():
    orientation = axis_angle_quat([1.0, 0.0, 0.0], 0.3)
    state = make_state(last_imu_orientation=orientation, current_depth=2.5)

    stabilization_functions.capture_callback(
        Empty(), state, {'use_setpoints': True}, {'use_setpoints': True}, MagicMock())

    assert np.allclose(
        quat_values(state['desired_orientation']), quat_values(orientation))
    assert state['desired_depth'] == pytest.approx(2.5)


def test_capture_callback_without_samples_keeps_setpoints():
    original = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    state = make_state(desired_orientation=original, desired_depth=1.0)
    logger = MagicMock()

    stabilization_functions.capture_callback(
        Empty(), state, {'use_setpoints': True}, {'use_setpoints': True}, logger)

    assert state['desired_orientation'] is original
    assert state['desired_depth'] == 1.0
    assert logger.warn.call_count == 2


def test_capture_callback_ignored_when_setpoints_disabled():
    original = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    state = make_state(
        desired_orientation=original,
        last_imu_orientation=axis_angle_quat([1.0, 0.0, 0.0], 0.3),
        current_depth=2.5,
    )

    stabilization_functions.capture_callback(Empty(), state, {}, {}, MagicMock())

    assert state['desired_orientation'] is original
    assert state['desired_depth'] is None


# publish_stabilization_commands

def test_publish_stabilization_commands():
    publisher = MagicMock()
    msg = Twist()

    stabilization_functions.publish_stabilization_commands(msg, publisher)

    publisher.publish.assert_called_once_with(msg)
