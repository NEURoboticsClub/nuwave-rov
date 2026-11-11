from geometry_msgs.msg import Quaternion, Twist
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float32


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def set_desired_orientation(q: Quaternion, state: dict, logger) -> bool:
    """Update the attitude setpoint; Input is normalized."""
    vals = np.array([q.w, q.x, q.y, q.z])
    if not np.all(np.isfinite(vals)):
        logger.warn(
            'Rejected desired orientation: quaternion has non-finite components'
        )
        return False

    n = float(np.linalg.norm(vals))
    if n < 1e-6:
        logger.warn(
            'Rejected desired orientation: quaternion has near-zero norm'
        )
        return False

    state['desired_orientation'] = Quaternion(
        w=q.w / n,
        x=q.x / n,
        y=q.y / n,
        z=q.z / n,
    )
    desired_orientation = state['desired_orientation']
    logger.info(
        f'Desired orientation set to '
        f'(w={desired_orientation.w:.3f}, '
        f'x={desired_orientation.x:.3f}, '
        f'y={desired_orientation.y:.3f}, '
        f'z={desired_orientation.z:.3f})'
    )
    return True


def _imu_to_twist_control(
    imu_msg: Imu,
    desired_orientation: Quaternion,
    imu_stabilization_params: dict,
    logger,
) -> Twist:
    """
    Compute a body-frame angular-velocity command.

    The command drives the current orientation toward the desired orientation.
    """
    cur = imu_msg.orientation
    des = desired_orientation
    gyro = imu_msg.angular_velocity

    q_cur = np.array([cur.w, cur.x, cur.y, cur.z])
    q_des = np.array([des.w, des.x, des.y, des.z])
    gyro_vec = np.array([gyro.x, gyro.y, gyro.z])

    # Reject malformed IMU data to never publish NaN/inf thruster
    if not (np.all(np.isfinite(q_cur)) and np.all(np.isfinite(gyro_vec))):
        logger.warn(
            'Ignoring IMU sample with non-finite orientation/gyro',
            throttle_duration_sec=5.0,
        )
        return Twist()

    n = np.linalg.norm(q_cur)
    if n < 1e-6:
        return Twist()
    q_cur /= n
    q_cur_inv = np.array([q_cur[0], -q_cur[1], -q_cur[2], -q_cur[3]])

    q_err_body = _quat_mul(q_cur_inv, q_des)
    if q_err_body[0] < 0.0:
        q_err_body = -q_err_body
    err_vec = 2.0 * q_err_body[1:]

    p = float(imu_stabilization_params.get('p', 1.0))
    d = float(imu_stabilization_params.get('d', 0.0))

    twist = Twist()
    twist.angular.x = float(p * err_vec[0] - d * gyro.x)
    twist.angular.y = float(p * err_vec[1] - d * gyro.y)
    twist.angular.z = float(p * err_vec[2] - d * gyro.z)

    max_influence = float(
        imu_stabilization_params.get('max_stabilization_influence', 1.0))
    v = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
    norm = np.linalg.norm(v)
    if norm > max_influence:
        v *= max_influence / norm
        twist.angular.x = float(v[0])
        twist.angular.y = float(v[1])
        twist.angular.z = float(v[2])

    twist.angular.z = 0.0

    return twist


def _rotate_vector(quaternion, vector):
    # Rotate a vector with q * [0, vector] * conjugate(q).
    conjugate = quaternion * np.array([1.0, -1.0, -1.0, -1.0])
    return _quat_mul(
        _quat_mul(quaternion, np.concatenate(([0.0], vector))), conjugate
    )[1:]


def update_xy_estimate(msg: Imu, state: dict, timeout_s: float):
    """Integrate body-frame position, assuming rest at startup or after a gap."""
    # Read body-frame acceleration, orientation, and sample time.
    orientation = msg.orientation
    quaternion = np.array([
        orientation.w, orientation.x, orientation.y, orientation.z,
    ])
    acceleration = np.array([
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
    ])
    sample_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    norm = np.linalg.norm(quaternion)
    # Discard the estimate and setpoint if required IMU data is invalid.
    if (
        not np.all(np.isfinite(quaternion))
        or not np.all(np.isfinite(acceleration))
        or norm < 1e-6
        or msg.orientation_covariance[0] == -1.0
        or msg.linear_acceleration_covariance[0] == -1.0
    ):
        state['current_position'] = None
        state['current_velocity'] = None
        state['desired_position'] = None
        state['xy_sample_time'] = None
        return

    # Normalize orientation and measure time since the previous sample.
    quaternion /= norm
    previous_time = state['xy_sample_time']
    delta_time = None if previous_time is None else sample_time - previous_time
    if delta_time is None or not 0.0 < delta_time <= timeout_s:
        # Restart from rest after initialization or a timestamp discontinuity.
        state['current_position'] = np.zeros(3)
        state['current_velocity'] = np.zeros(3)
        state['desired_position'] = None
    else:
        # Express stored vectors in the current body axes.
        inverse = quaternion * np.array([1.0, -1.0, -1.0, -1.0])
        rotation = _quat_mul(inverse, state['xy_orientation'])
        velocity = _rotate_vector(rotation, state['current_velocity'])
        # Integrate velocity and acceleration into position.
        state['current_position'] = (
            _rotate_vector(rotation, state['current_position'])
            + velocity * delta_time
            + 0.5 * acceleration * delta_time ** 2
        )
        # Integrate acceleration into velocity.
        state['current_velocity'] = velocity + acceleration * delta_time
        # Keep the captured target in the same body axes as the estimate.
        if state['desired_position'] is not None:
            state['desired_position'] = _rotate_vector(rotation, state['desired_position'])

    # Retain this sample as the reference for the next update.
    state['xy_orientation'] = quaternion
    state['xy_sample_time'] = sample_time


def _xy_to_twist_control(
    current_position,
    current_velocity,
    desired_position,
    xy_stabilization_params: dict,
) -> Twist:
    """Compute bounded horizontal PD corrections in the body frame."""
    twist = Twist()
    # Pull toward the XY setpoint and damp horizontal velocity.
    proportional = float(xy_stabilization_params.get('p', 0.0))
    derivative = float(xy_stabilization_params.get('d', 0.0))
    command = (
        proportional * (desired_position[:2] - current_position[:2])
        - derivative * current_velocity[:2]
    )
    if not np.all(np.isfinite(command)):
        return twist
    # Limit combined XY output without changing its direction.
    maximum = max(0.0, float(
        xy_stabilization_params.get('max_stabilization_influence', 0.0)
    ))
    magnitude = np.linalg.norm(command)
    if magnitude > maximum:
        command *= maximum / magnitude
    twist.linear.x = float(command[0])
    twist.linear.y = float(command[1])
    return twist


def _depth_to_twist_control(
    current_depth: float,
    depth_rate: float,
    desired_depth: float,
    depth_stabilization_params: dict,
) -> float:
    """Compute the normalized vertical command from depth error and rate."""
    p = float(depth_stabilization_params.get('p', 1.0))
    d = float(depth_stabilization_params.get('d', 0.0))
    command = p * (current_depth - desired_depth) + d * depth_rate
    max_influence = max(
        0.0,
        float(depth_stabilization_params.get('max_stabilization_influence', 1.0)),
    )
    return float(np.clip(command, -max_influence, max_influence))


def depth_control(
    state: dict,
    manual_vertical_active: bool,
    depth_stabilization_params: dict,
) -> float:
    """Suspend depth hold during manual movement and capture depth on release."""
    if manual_vertical_active:
        return 0.0

    if state.get('depth_capture_pending', False):
        state['desired_depth'] = state['current_depth']
        state['depth_capture_pending'] = False

    return _depth_to_twist_control(
        state['current_depth'],
        state['depth_rate'],
        state['desired_depth'],
        depth_stabilization_params,
    )


def depth_callback(
    msg: Float32,
    state: dict,
    now: float,
    depth_timeout_s: float,
    velocity_filter_alpha: float,
    logger,
):
    depth = float(msg.data)
    if not np.isfinite(depth):
        logger.warn(
            'Ignoring depth sample with non-finite value',
            throttle_duration_sec=5.0,
        )
        return

    previous_depth = state['current_depth']
    previous_time = state['depth_sample_time']
    depth_rate = 0.0
    if previous_depth is not None and previous_time is not None:
        delta_time = now - previous_time
        if 0.0 < delta_time <= depth_timeout_s:
            raw_depth_rate = (depth - previous_depth) / delta_time
            alpha = float(np.clip(velocity_filter_alpha, 0.0, 1.0))
            depth_rate = (
                alpha * raw_depth_rate
                + (1.0 - alpha) * state['depth_rate']
            )

    state['current_depth'] = depth
    state['depth_rate'] = depth_rate
    state['depth_sample_time'] = now


def imu_callback(
    msg: Imu,
    state: dict,
    now: float,
):
    """Cache the latest IMU sample for the combined control update."""
    state['last_imu_orientation'] = msg.orientation
    state['last_imu_msg'] = msg
    state['imu_sample_time'] = now


def capture_callback(
    msg: Empty,
    state: dict,
    imu_stabilization_params: dict,
    depth_stabilization_params: dict,
    logger,
    xy_stabilization_params: dict | None = None,
):
    if imu_stabilization_params.get('use_setpoints', False):
        if state['last_imu_orientation'] is None:
            logger.warn(
                'Capture requested but no IMU received yet; orientation setpoint unchanged'
            )
        elif set_desired_orientation(state['last_imu_orientation'], state, logger):
            logger.info('Captured current orientation as setpoint')

    if depth_stabilization_params.get('use_setpoints', False):
        if state['current_depth'] is None:
            logger.warn(
                'Capture requested but no depth received yet; depth setpoint unchanged'
            )
        else:
            state['desired_depth'] = state['current_depth']
            logger.info(f'Captured current depth as setpoint: {state["desired_depth"]:.3f} m')

            # Capture an independent position target without resetting velocity.
    if (xy_stabilization_params or {}).get('use_setpoints', False):
        if state['current_position'] is None:
            logger.warn('Capture requested but no XY estimate available; setpoint unchanged')
        else:
            state['desired_position'] = state['current_position'].copy()
            logger.info('Captured current XY position as setpoint')


def publish_stabilization_commands(msg: Twist, publisher):
    """Publish new Twist commands to houston."""
    publisher.publish(msg)
