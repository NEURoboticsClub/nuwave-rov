"""Functions used by the arm controller node."""

import numpy as np
from std_msgs.msg import Float32


def status_callback(msg, motors, logger, clock):
    """Process a new velocity vector and turns them into PWM signals for the Arm Motors."""
    last_msg_time = clock.now()

    arm_velocities = np.array(msg.data, dtype=float)

    for mot in motors:
        idx = mot.get('id', 0) - 1
        if 0 <= idx < len(arm_velocities):
            vel = arm_velocities[idx]
            mot['dutycycle'] = vel if np.isfinite(vel) else 0.0
        else:
            logger.warn(
                f'Arm command array has {len(arm_velocities)} elements; '
                f"no value for motor id {mot.get('id')}; holding zero"
            )
            mot['dutycycle'] = 0.0

    return last_msg_time


def publish_arm_motors(
        motors, clock, logger, watchdog_timeout_s, last_msg_time, watchdog_tripped):
    """Publish current PWM commands to each arm motor."""
    timed_out = (
        last_msg_time is None or
        (clock.now() - last_msg_time).nanoseconds * 1e-9
        > watchdog_timeout_s
    )
    # Log once per timeout event
    if timed_out:
        if last_msg_time is not None and not watchdog_tripped:
            logger.warn(
                f'Watchdog timeout: no arm command in {watchdog_timeout_s}s, '
                'zeroing arm motor outputs.'
            )
            watchdog_tripped = True
    else:
        watchdog_tripped = False
    for mot in motors:
        msg = Float32()
        msg.data = 0.0 if timed_out else float(mot.get('dutycycle', 0.0))
        mot['pub'].publish(msg)

    return watchdog_tripped