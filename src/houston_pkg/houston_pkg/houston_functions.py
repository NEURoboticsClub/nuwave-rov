import numpy as np
from std_msgs.msg import Bool, Empty, Float32MultiArray
from geometry_msgs.msg import Twist

def scale_controller_input(x:float) -> float :
    if abs(x) <= 0.05:
        return 0.0

    abs_x = abs(x)
    result = np.sign(x) * ((1.2 * np.power(1.0356, abs_x * 100.0)) - 1.2 + (0.2 * abs_x * 100.0))

    # Normalize curve output back to [-1, 1].
    max_result = (1.2 * np.power(1.0356, 100.0)) - 1.2 + (0.2 * 100.0)
    if max_result <= 0:
        return float(x)
    return float(np.clip(result / max_result, -1.0, 1.0))


def parse_joystick(cfg_list, msg, is_expo_enabled) -> dict:

    if not isinstance(cfg_list, list):
        return {"axis": {}, "button": {}} # config malformed

    axis_values = {}
    button_values = {}
    for cfg in cfg_list:
        # --- Axis entry ---
        if "axis" in cfg:
            axis_name = cfg["axis"]
            axis_index = int(cfg.get("input", 0))
            invert = bool(cfg.get("invert", False))
            sensitivity = float(cfg.get("sensitivity", 1.0))
            scale = cfg.get("scale", "linear")
            # Controller is NOISEY! Tune deadzone, so that stick drift still gives us neutral when we have it at rest
            # Clamp below 1.0 so the rescaling can't divide by zero
            deadzone = min(max(float(cfg.get("deadzone", 0.07)), 0.0), 0.99)

            raw = msg.axes[axis_index] if axis_index < len(msg.axes) else 0.0
            if invert:
                raw *= -1.0
            
            # Deadzone processing
            if abs(raw) < deadzone:
                raw = 0.0
            else:
                # Deadzone rescaling, so it is still within the [-1, 1] range
                # This is so it doesn't go 0.0 to +- deadzone immediately.
                raw = (raw - np.sign(raw) * deadzone) / (1.0 - deadzone)

            val = raw * sensitivity

            # Optional: support your "scale" field (keep simple)
            if scale == "logarithmic":
                # compress near 0, preserve sign
                val = np.sign(val) * np.log1p(abs(val))
            elif scale == "exponential" and is_expo_enabled:
                val = scale_controller_input(val)

            val = float(np.clip(val, -1.0, 1.0))

            axis_values[axis_name] = float(val)
            continue

        # --- Button entry ---
        if "button" in cfg:
            btn_name = cfg["button"]
            btn_index = int(cfg.get("input", 0))
            invert = bool(cfg.get("invert", False))

            raw = msg.buttons[btn_index] if btn_index < len(msg.buttons) else 0
            pressed = (1 - raw) if invert else raw
            button_values[btn_name] = int(pressed)
            continue

    return {"axis": axis_values, "button": button_values}



def publish_boolean(is_enabled, pub_function):
    msg = Bool()
    msg.data = bool(is_enabled)
    pub_function.publish(msg)



def joy_arm_callback(msg):
    return_map = parse_joystick(msg, cfg_type="arm_control")
    # self.get_logger().info(str(return_map))
    return return_map

def publish_twist_get_msg(axis_values, button_values):
    msg = Twist()
    msg.angular.x = float(axis_values.get('pitch', 0.0))
    msg.angular.y = float(button_values.get('roll_right', 0.0)) - float(button_values.get('roll_left', 0.0))
    msg.angular.z = float(axis_values.get('yaw', 0.0))


    msg.linear.x = float(axis_values.get('strafe', 0.0))
    msg.linear.y = float(axis_values.get('drive_forward', 0.0))
    msg.linear.z = (float(axis_values.get('up', 0.0)) - float(axis_values.get('down', 0.0))) * 0.5
    return msg

def publish_arm_commands(axis_values, button_values):
    msg = Float32MultiArray()
    msg.data = [
        float(axis_values.get('base_yaw_input', 0.0)),
        float(axis_values.get('base_pitch_input', 0.0)),
        float(axis_values.get('elbow_pitch_input', 0.0)),
        float(axis_values.get('wrist_yaw_input', 0.0)),
        (float(axis_values.get('wrist_up_input', 0.0)) - float(axis_values.get('wrist_down_input', 0.0))) * 0.5,
        (float(button_values.get('claw_open_input', 0.0)) - float(button_values.get('claw_close_input', 0.0))), 
    ]
    return msg




def _scale_to_unit(x, y, z):
    m = max(abs(x), abs(y), abs(z), 1.0)
    return x / m, y / m, z / m

def set_button_enabled(button_values, toggle, prev_button, set_enabled_function, already_enabled):
    btn = int(button_values.get(toggle, 0))
    if btn == 1 and prev_button == 0:
        set_enabled_function(not already_enabled, 'controller')
    prev_button = btn

def check_if_stale_and_set_msg(clock, last_stabilizer_time, last_stabilizer_twist, stabilizer_timeout, set_stabilize_enabled, logger, msg):
    now = clock.now()
    stale = (
        last_stabilizer_time is not None
        and (now - last_stabilizer_time).nanoseconds * 1e-9 > stabilizer_timeout
    )
    if stale:
        set_stabilize_enabled(False, 'stale_data')
        logger.warn(
            "Stabilizer messages stale; stabilization disabled"
        )
    elif last_stabilizer_time is not None:
        s = last_stabilizer_twist
        lx, ly, lz = _scale_to_unit(
            msg.linear.x + s.linear.x,
            msg.linear.y + s.linear.y,
            msg.linear.z + s.linear.z,
        )
        ax, ay, az = _scale_to_unit(
            msg.angular.x + s.angular.x,
            msg.angular.y + s.angular.y,
            msg.angular.z + s.angular.z,
        )
        msg.linear.x, msg.linear.y, msg.linear.z = lx, ly, lz
        msg.angular.x, msg.angular.y, msg.angular.z = ax, ay, az

def publish_loop(last_joy_thruster_msg, clock, last_joy_thruster_time, joy_thruster_timeout, joy_thruster_stale, logger, twist_pub):
    if last_joy_thruster_msg is None:
            return

    # Watchdog to prevent runaway if houston stops reveiving joystick messages
    now = clock.now()
    joy_stale = (
        last_joy_thruster_time is not None
        and (now - last_joy_thruster_time).nanoseconds * 1e-9 > joy_thruster_timeout
    )
    if joy_stale:
        if not joy_thruster_stale:
            logger.warn(
                "Thruster joystick input stale; publishing neutral twist"
            )
            joy_thruster_stale = True
        twist_pub.publish(Twist())
        return joy_thruster_stale
    if joy_thruster_stale:
        logger.info("Thruster joystick input restored")
        joy_thruster_stale = False
    return joy_thruster_stale

def try_parse_joystick_publish(last_joy_thruster_msg, parse_joystick, publish_twist, logger):
    try:
        parsed = parse_joystick(last_joy_thruster_msg, cfg_type="thruster_control")
        # self.get_logger().info(str(parsed))

        # Publish the result
        publish_twist(parsed["axis"], parsed["button"])
        return 
    except Exception as e:
        logger.error(f"Error processing thruster joystick input: {e}")