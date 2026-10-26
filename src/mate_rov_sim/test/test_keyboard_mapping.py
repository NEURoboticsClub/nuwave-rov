"""
Runs keyboard_teleop's Joy messages through houston's own parser and joystick
config, so a change to houston_pkg/config/joystick_config.yaml can't silently
break (or flip) a key.
"""
import os
from types import SimpleNamespace

import pytest
import yaml

from mate_rov_sim.keyboard_teleop import ARM_KEYS, THRUSTER_KEYS, JoyLayout

houston = pytest.importorskip('houston_pkg.houston_node')

JOY_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                          'houston_pkg', 'config', 'joystick_config.yaml')


@pytest.fixture(scope='module')
def config():
    if not os.path.exists(JOY_CONFIG):
        pytest.skip(f'{JOY_CONFIG} not found')
    with open(JOY_CONFIG) as f:
        return yaml.safe_load(f)


def parse(config, section, keymap, keys):
    """What houston sees for these held keys: (axis values, button values)."""
    layout = JoyLayout(config[section])
    fake_houston = SimpleNamespace(joy_map=config, expo_enabled=False)
    out = houston.Houston.parse_joystick(fake_houston, layout.build(set(keys), keymap), section)
    return out['axis'], out['button']


def twist(axis, button):
    """houston.publish_twist's mapping (without stabilization)."""
    return {
        'linear.x': axis.get('strafe', 0.0),
        'linear.y': axis.get('drive_forward', 0.0),
        'linear.z': (axis.get('up', 0.0) - axis.get('down', 0.0)) * 0.5,
        'angular.x': axis.get('pitch', 0.0),
        'angular.y': button.get('roll_right', 0) - button.get('roll_left', 0),
        'angular.z': axis.get('yaw', 0.0),
    }


@pytest.mark.parametrize('key, field, expected', [
    ('w', 'linear.y', 1.0), ('s', 'linear.y', -1.0),
    ('d', 'linear.x', 1.0), ('a', 'linear.x', -1.0),      # x = right
    ('r', 'linear.z', 1.0), ('f', 'linear.z', -1.0),
    ('q', 'angular.z', 1.0), ('e', 'angular.z', -1.0),    # +z = counter-clockwise = yaw left
    ('Up', 'angular.x', -1.0), ('Down', 'angular.x', 1.0),
    ('Right', 'angular.y', 1), ('Left', 'angular.y', -1),
])
def test_rov_keys(config, key, field, expected):
    t = twist(*parse(config, 'thruster_control', THRUSTER_KEYS, [key]))
    assert t[field] == pytest.approx(expected)
    assert all(v == 0 for f, v in t.items() if f != field), f'{key} also moved {t}'


def test_no_keys_is_neutral(config):
    axis, button = parse(config, 'thruster_control', THRUSTER_KEYS, [])
    assert all(v == 0 for v in twist(axis, button).values())
    assert not any(button.values())


@pytest.mark.parametrize('key, button', [('p', 'precision_toggle'), ('x', 'stabilize_toggle'),
                                         ('z', 'expo_toggle')])
def test_toggle_keys(config, key, button):
    assert parse(config, 'thruster_control', THRUSTER_KEYS, [key])[1][button] == 1


def arm_commands(axis, button):
    """houston.publish_arm_commands' mapping."""
    return [
        axis.get('base_yaw_input', 0.0),
        axis.get('base_pitch_input', 0.0),
        axis.get('elbow_pitch_input', 0.0),
        axis.get('wrist_yaw_input', 0.0),
        (axis.get('wrist_up_input', 0.0) - axis.get('wrist_down_input', 0.0)) * 0.5,
        button.get('claw_open_input', 0) - button.get('claw_close_input', 0),
    ]


@pytest.mark.parametrize('key, index, expected', [
    ('j', 0, 1.0), ('l', 0, -1.0), ('i', 1, 1.0), ('k', 1, -1.0), ('u', 2, 1.0), ('o', 2, -1.0),
    ('n', 3, 1.0), ('m', 3, -1.0), ('y', 4, 1.0), ('h', 4, -1.0),
    ('bracketleft', 5, 1), ('bracketright', 5, -1),
])
def test_arm_keys(config, key, index, expected):
    cmd = arm_commands(*parse(config, 'arm_control', ARM_KEYS, [key]))
    assert cmd[index] == pytest.approx(expected)
    assert all(v == 0 for i, v in enumerate(cmd) if i != index), f'{key} also moved {cmd}'


def test_opposite_keys_cancel(config):
    t = twist(*parse(config, 'thruster_control', THRUSTER_KEYS, ['w', 's']))
    assert t['linear.y'] == 0.0
