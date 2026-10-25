import numpy as np
import pytest
from stabilization_pkg.stabilization_pkg import stabilization_functions

## Test quat mul
def test_quat_mul():
    q1 = np.array([0.7071, 0.7071, 0, 0])
    q2 = np.array([0.7071, 0, 0.7071, 0])
    expected_result = np.array([0.5, 0.5, 0.5, -0.5])
    result = stabilization_functions._quat_mul(q1, q2)
    assert np.allclose(result, expected_result), f"Expected {expected_result}, but got {result}"

## Test set_desired_orientation
def test_desired_orientation():
    state = {
        'desired_orientation': np.array([1, 0, 0, 0]),
        'last_imu_orientation': np.array([1, 0, 0, 0])
    }
    q_des = np.array([0.7071, 0.7071, 0, 0])
    success = stabilization_functions.set_desired_orientation(q_des, state, None)
    assert success
    assert np.allclose(state['desired_orientation'], q_des), f"Expected {q_des}, but got {state['desired_orientation']}"


class Test_ImuToTwistControl:

    ## Test _imu_to_twist_control
    def test_imu_to_twist_control():
        imu_msg = type('Imu', (object,), {
            'orientation': type('Orientation', (object,), {'x': 0.7071, 'y': 0.7071, 'z': 0, 'w': 0})(),
            'angular_velocity': type('AngularVelocity', (object,), {'x': 0.1, 'y': 0.2, 'z': 0.3})()
        })()
        desired_orientation = np.array([1, 0, 0, 0])
        imu_stabilization_params = {'p': 1.0, 'd': 0.5, 'max_stabilization_influence': 1.0}
        twist = stabilization_functions._imu_to_twist_control(imu_msg, desired_orientation, imu_stabilization_params, None)
        assert isinstance(twist, type('Twist', (object,), {'angular': type('Angular', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()})())), "Expected a Twist object"

    def test_imu_to_twist_control_with_non_finite_data():
        imu_msg = type('Imu', (object,), {
            'orientation': type('Orientation', (object,), {'x': np.nan, 'y': 0.7071, 'z': 0, 'w': 0})(),
            'angular_velocity': type('AngularVelocity', (object,), {'x': 0.1, 'y': 0.2, 'z': 0.3})()
        })()
        desired_orientation = np.array([1, 0, 0, 0])
        imu_stabilization_params = {'p': 1.0, 'd': 0.5}
        twist = stabilization_functions._imu_to_twist_control(imu_msg, desired_orientation, imu_stabilization_params, None)
        assert isinstance(twist, type('Twist', (object,), {'angular': type('Angular', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()})())), "Expected a Twist object"
        assert twist.angular.x == 0 and twist.angular.y == 0 and twist.angular.z == 0, "Expected zero angular velocities for non-finite IMU data"

    def test_imu_to_twist_control_with_zero_norm():
        imu_msg = type('Imu', (object,), {
            'orientation': type('Orientation', (object,), {'x': 0, 'y': 0, 'z': 0, 'w': 0})(),
            'angular_velocity': type('AngularVelocity', (object,), {'x': 0.1, 'y': 0.2, 'z': 0.3})()
        })()
        desired_orientation = np.array([1, 0, 0, 0])
        imu_stabilization_params = {'p': 1.0, 'd': 0.5}
        twist = stabilization_functions._imu_to_twist_control(imu_msg, desired_orientation, imu_stabilization_params, None)
        assert isinstance(twist, type('Twist', (object,), {'angular': type('Angular', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()})())), "Expected a Twist object"
        assert twist.angular.x == 0 and twist.angular.y == 0 and twist.angular.z == 0, "Expected zero angular velocities for zero-norm orientation"

    def test_imu_to_twist_control_with_negative_error():
        imu_msg = type('Imu', (object,), {
            'orientation': type('Orientation', (object,), {'x': 0.7071, 'y': 0.7071, 'z': 0, 'w': 0})(),
            'angular_velocity': type('AngularVelocity', (object,), {'x': 0.1, 'y': 0.2, 'z': 0.3})()
        })()
        desired_orientation = np.array([-1, 0, 0, 0])  # This will create a negative error
        imu_stabilization_params = {'p': 1.0, 'd': 0.5}
        twist = stabilization_functions._imu_to_twist_control(imu_msg, desired_orientation, imu_stabilization_params, None)
        assert isinstance(twist, type('Twist', (object,), {'angular': type('Angular', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()})())), "Expected a Twist object"

def test_depth_callback():
    msg = type('Float32', (object,), {'data': 5.0})()
    state = {
        'current_depth': 4.0,
        'depth_rate': 0.1,
        'desired_depth': 5.0,
        'last_depth_msg_time': None
    }
    now = 1234567890.0
    depth_timeout_s = 1.0
    velocity_filter_alpha = 0.5
    stabilization_functions.depth_callback(msg, state, now, depth_timeout_s, velocity_filter_alpha, None)
    assert state['current_depth'] == msg.data, "Expected current_depth to be updated to the message data"

def test_imu_callback():
    imu_msg = type('Imu', (object,), {
        'orientation': type('Orientation', (object,), {'x': 0.7071, 'y': 0.7071, 'z': 0, 'w': 0})(),
        'angular_velocity': type('AngularVelocity', (object,), {'x': 0.1, 'y': 0.2, 'z': 0.3})()
    })()
    state = {
        'last_imu_orientation': np.array([1, 0, 0, 0]),
        'last_imu_msg': None,
        'imu_sample_time': None
    }
    now = 1234567890.0
    stabilization_functions.imu_callback(imu_msg, state, now)
    assert np.allclose(state['last_imu_orientation'], [imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z]), "Expected last_imu_orientation to match the IMU message orientation"

def test_capture_callback():
    state = {
        'last_imu_orientation': np.array([1, 0, 0, 0]),
        'last_imu_msg': None,
        'imu_sample_time': None
    }
    now = 1234567890.0
    stabilization_functions.capture_callback(state, now)
    assert state['imu_sample_time'] == now, "Expected imu_sample_time to be updated to the current time"

def test_capture_callback_with_non_finite_time():
    state = {
        'last_imu_orientation': np.array([1, 0, 0, 0]),
        'last_imu_msg': None,
        'imu_sample_time': None
    }
    now = float('nan')  # Non-finite time
    stabilization_functions.capture_callback(state, now)
    assert state['imu_sample_time'] is None, "Expected imu_sample_time to remain None for non-finite time"

def test_publish_stabilization_command():
    state = {
        'last_imu_orientation': np.array([1, 0, 0, 0]),
        'last_imu_msg': None,
        'imu_sample_time': None
    }
    now = 1234567890.0
    stabilization_functions.publish_stabilization_command(state, now)
    assert state['last_imu_msg'] is not None, "Expected last_imu_msg to be updated with a new Twist message"
