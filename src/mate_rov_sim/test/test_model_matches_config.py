"""
The simulator duplicates a few facts from the NUWave configs (thruster geometry,
motor topics). These tests fail if they drift apart.

  colcon test --packages-select mate_rov_sim && colcon test-result --verbose
  # or, from this package's folder:  python3 -m pytest test
"""
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from mate_rov_sim.sim_motor_interface import joint_name_for

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(PKG)
MODEL = os.path.join(PKG, 'models', 'mate_rov', 'model.sdf')
BRIDGE = os.path.join(PKG, 'config', 'bridge.yaml')
THRUSTER_CONFIG = os.path.join(SRC, 'thruster_controller_pkg', 'config', 'thruster_config.yaml')
MOTOR_CONFIG = os.path.join(SRC, 'thruster_pkg', 'config', 'individual_motor_config.yaml')


def load(path):
    if not os.path.exists(path):
        pytest.skip(f'{path} not found (run from the nuwave-rov source tree)')
    with open(path) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def model():
    return ET.parse(MODEL).getroot().find('model')


def floats(text):
    return np.array([float(v) for v in text.split()])


def test_thrusters_match_controller_config(model):
    thrusters = load(THRUSTER_CONFIG)['thrusters']
    links = {link.get('name'): link for link in model.findall('link')}
    joints = {joint.get('name'): joint for joint in model.findall('joint')}
    plugins = {p.findtext('joint_name') for p in model.findall('plugin')
               if p.get('name') == 'gz::sim::systems::Thruster'}
    for t in thrusters:
        joint = joint_name_for(t['topic'])
        link_name = joint.removesuffix('_joint')
        assert link_name in links, f'no link {link_name} for {t["topic"]}'
        pose = links[link_name].find('pose')
        assert pose.get('relative_to') == 'base_link'
        p = floats(pose.text)
        assert np.allclose(p[:3], t['position_m'], atol=1e-4), f'{link_name} position'
        assert np.allclose(p[3:], 0.0), f'{link_name} must not be rotated (axis is in base_link frame)'

        d = np.array(t['direction'], dtype=float)
        axis = floats(joints[joint].find('axis/xyz').text)
        assert np.allclose(axis / np.linalg.norm(axis), d / np.linalg.norm(d), atol=1e-4), f'{joint} axis'
        assert joint in plugins, f'no Thruster plugin for {joint}'
    assert len(plugins) == len(thrusters)


def test_every_motor_has_a_joint_and_bridge(model):
    motors = load(MOTOR_CONFIG)['motors']
    joints = {joint.get('name') for joint in model.findall('joint')}
    bridged = {e['ros_topic_name'] for e in load(BRIDGE)}
    for mot in motors:
        joint = joint_name_for(mot['topic'])
        assert joint is not None, f'{mot["topic"]} is not under /thruster/ or /arm/'
        assert joint in joints, f'model.sdf has no joint {joint} for {mot["topic"]}'
        assert f'/sim/{joint}/cmd' in bridged, f'bridge.yaml does not carry /sim/{joint}/cmd'


def test_arm_joints_have_velocity_controllers(model):
    controlled = {p.findtext('joint_name') for p in model.findall('plugin')
                  if p.get('name') == 'gz::sim::systems::JointController'}
    arm_joints = {j.get('name') for j in model.findall('joint') if j.get('name').startswith('arm_')}
    assert arm_joints and arm_joints == controlled


def test_cameras_are_bridged(model):
    sensors = model.findall('link/sensor[@type="camera"]')
    bridged = {e['gz_topic_name'] for e in load(BRIDGE)}
    params = load(os.path.join(PKG, 'config', 'rov_params.yaml'))
    ids = params['camera_sim']['ros__parameters']['camera_ids']
    assert sorted(s.get('name') for s in sensors) == [f'camera_{i}' for i in sorted(ids)]
    for s in sensors:
        assert '/' + s.findtext('topic') in bridged
