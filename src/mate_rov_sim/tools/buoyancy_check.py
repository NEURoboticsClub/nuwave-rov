#!/usr/bin/env python3
"""
buoyancy_check.py: sanity-check the trim of an SDF underwater vehicle BEFORE you run Gazebo.

It reads the model the same way Gazebo's Buoyancy system does (every <collision>
displaces water at its center) and reports:
  * total mass and center of mass (CoM)
  * displaced volume, buoyant force, net buoyancy, center of buoyancy (CoB)
  * whether the vehicle will float, sink, tip or flip
  * (optional) the buoyancy-box height that gives a target net buoyancy
  * (optional) quadratic drag estimates from the frame size

No ROS or Gazebo needed: plain Python 3 + numpy.

Examples:
  python3 tools/buoyancy_check.py models/mate_rov/model.sdf
  python3 tools/buoyancy_check.py models/mate_rov/model.sdf --target-net 0.10
  python3 tools/buoyancy_check.py models/mate_rov/model.sdf --frame 0.45 0.35 0.28 --cd 1.0

Limitations: supports box, sphere, cylinder, capsule and ellipsoid collisions,
link poses relative to the model or to another link, and no nested models.
Mesh collisions are reported and skipped (Gazebo's graded buoyancy skips them too).
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def parse_pose(elem):
    """Return (4x4 transform, relative_to) for an element's <pose> child."""
    t = np.eye(4)
    rel = None
    if elem is not None:
        pose = elem.find('pose')
        if pose is not None and pose.text and pose.text.strip():
            v = [float(x) for x in pose.text.split()]
            v += [0.0] * (6 - len(v))
            t[:3, :3] = rpy_to_matrix(*v[3:6])
            t[:3, 3] = v[:3]
            rel = pose.get('relative_to')
    return t, rel


def shape_volume(geom):
    """Volume (m^3) of a collision geometry, or (None, name) if unsupported."""
    for child in geom:
        tag = child.tag
        if tag == 'box':
            sx, sy, sz = (float(x) for x in child.find('size').text.split())
            return sx * sy * sz, 'box'
        if tag == 'sphere':
            r = float(child.find('radius').text)
            return 4.0 / 3.0 * math.pi * r ** 3, 'sphere'
        if tag == 'cylinder':
            r = float(child.find('radius').text)
            h = float(child.find('length').text)
            return math.pi * r ** 2 * h, 'cylinder'
        if tag == 'capsule':
            r = float(child.find('radius').text)
            h = float(child.find('length').text)
            return math.pi * r ** 2 * h + 4.0 / 3.0 * math.pi * r ** 3, 'capsule'
        if tag == 'ellipsoid':
            a, b, c = (float(x) for x in child.find('radii').text.split())
            return 4.0 / 3.0 * math.pi * a * b * c, 'ellipsoid'
        return None, tag
    return None, 'empty'


def link_transforms(model):
    """Model-frame transform of every link (handles relative_to another link)."""
    raw = {}
    for link in model.findall('link'):
        raw[link.get('name')] = parse_pose(link)
    done = {}

    def resolve(name, depth=0):
        if name in done:
            return done[name]
        if depth > 20:
            sys.exit('pose relative_to loop involving link ' + name)
        t, rel = raw[name]
        if rel and rel != '__model__':
            if rel not in raw:
                sys.exit(f'link {name}: relative_to="{rel}" is not a link (frames not supported here)')
            t = resolve(rel, depth + 1) @ t
        done[name] = t
        return t

    for name in raw:
        resolve(name)
    return done


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sdf', help='path to model.sdf')
    ap.add_argument('--density', type=float, default=997.0, help='water density kg/m^3 (default 997, fresh pool)')
    ap.add_argument('--g', type=float, default=9.81)
    ap.add_argument('--target-net', type=float, metavar='KG',
                    help='desired net buoyancy in kg (e.g. 0.1 = 100 g positive); prints the box height to use')
    ap.add_argument('--tune', default='buoyancy_volume',
                    help='name of the box collision to resize for --target-net (default buoyancy_volume)')
    ap.add_argument('--frame', type=float, nargs=3, metavar=('L', 'W', 'H'),
                    help='overall frame length (front to back), width, height in m: prints quadratic drag estimates')
    ap.add_argument('--cd', type=float, default=1.0, help='drag coefficient for --frame (default 1.0)')
    a = ap.parse_args()

    root = ET.parse(a.sdf).getroot()
    model = root.find('model') if root.tag == 'sdf' else root
    if model is None or model.tag != 'model':
        sys.exit('no <model> found in ' + a.sdf)

    tf = link_transforms(model)
    total_mass, mass_moment = 0.0, np.zeros(3)
    total_vol, vol_moment = 0.0, np.zeros(3)
    rows, tune_box = [], None

    for link in model.findall('link'):
        name = link.get('name')
        t_link = tf[name]
        inertial = link.find('inertial')
        if inertial is not None and inertial.find('mass') is not None:
            m = float(inertial.find('mass').text)
            t_i, _ = parse_pose(inertial)
            com = (t_link @ t_i)[:3, 3]
            total_mass += m
            mass_moment += m * com
        for col in link.findall('collision'):
            t_c, _ = parse_pose(col)
            center = (t_link @ t_c)[:3, 3]
            vol, kind = shape_volume(col.find('geometry'))
            cname = f'{name}/{col.get("name")}'
            if vol is None:
                rows.append((cname, kind, 0.0, center, 'SKIPPED (unsupported shape)'))
                continue
            total_vol += vol
            vol_moment += vol * center
            rows.append((cname, kind, vol, center, ''))
            if col.get('name') == a.tune and kind == 'box':
                size = [float(x) for x in col.find('geometry/box/size').text.split()]
                tune_box = (size, vol)

    if total_mass <= 0:
        sys.exit('model has no mass')
    com = mass_moment / total_mass
    cob = vol_moment / total_vol if total_vol > 0 else np.zeros(3)
    buoy_kg = total_vol * a.density
    net_kg = buoy_kg - total_mass

    print(f'\nModel: {a.sdf}   water density {a.density:.0f} kg/m^3\n')
    print(f'{"collision":34s} {"shape":9s} {"volume (L)":>10s}   center (m)')
    for cname, kind, vol, c, note in rows:
        print(f'{cname:34s} {kind:9s} {vol * 1000:10.3f}   ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f}) {note}')
    print()
    print(f'Total mass ............ {total_mass:8.3f} kg')
    print(f'Displaced volume ...... {total_vol * 1000:8.3f} L')
    print(f'Buoyancy .............. {buoy_kg:8.3f} kg-force  ({buoy_kg * a.g:.1f} N)')
    sign = 'POSITIVE (floats)' if net_kg > 0 else 'NEGATIVE (sinks)' if net_kg < 0 else 'neutral'
    print(f'Net buoyancy .......... {net_kg:+8.3f} kg-force  ({net_kg * a.g:+.2f} N)  {sign}')
    print(f'Center of mass ........ ({com[0]:+.4f}, {com[1]:+.4f}, {com[2]:+.4f}) m')
    print(f'Center of buoyancy .... ({cob[0]:+.4f}, {cob[1]:+.4f}, {cob[2]:+.4f}) m')
    d = cob - com
    print(f'CoB above CoM ......... {d[2] * 100:+.1f} cm')

    print('\nVerdict:')
    if abs(net_kg) > 0.5:
        print(f'  ! Net buoyancy is {net_kg:+.2f} kg. MATE ROVs are usually trimmed within a few hundred grams.')
    elif net_kg < 0:
        print('  ! Slightly negative: the ROV will sink when thrusters are off.')
    else:
        print('  OK: slightly positive, it will drift up to the surface when thrusters are off.')
    if d[2] <= 0:
        print('  ! CoB is NOT above CoM: the ROV will flip over. Move buoyancy up or mass down.')
    elif d[2] < 0.01:
        print('  ! CoB is less than 1 cm above CoM: very weak self-righting.')
    else:
        print(f'  OK: self-righting (CoB {d[2] * 100:.1f} cm above CoM).')
    horiz = math.hypot(d[0], d[1])
    if horiz > 0.005:
        dz = max(d[2], 1e-6)
        msgs = []
        # NUWave frame: x = right, y = forward
        if abs(d[1]) > 0.003:
            msgs.append(f'nose {"UP" if d[1] > 0 else "DOWN"} about {math.degrees(math.atan(abs(d[1]) / dz)):.0f} deg')
        if abs(d[0]) > 0.003:
            msgs.append(f'{"left" if d[0] > 0 else "right"} side low about {math.degrees(math.atan(abs(d[0]) / dz)):.0f} deg')
        print(f'  ! CoB is {horiz * 100:.1f} cm off-center horizontally: expect it to rest tilted ({", ".join(msgs)}).')
    else:
        print('  OK: CoB is centered over CoM, it should rest level.')

    if a.target_net is not None:
        if tune_box is None:
            print(f'\n--target-net: no box collision named "{a.tune}" found.')
        else:
            (sx, sy, sz), v = tune_box
            needed = (total_mass + a.target_net) / a.density - (total_vol - v)
            print(f'\nFor net buoyancy {a.target_net:+.3f} kg, set {a.tune} to size '
                  f'"{sx:g} {sy:g} {needed / (sx * sy):.4f}"  (height {needed / (sx * sy) * 100:.1f} cm).')
            print('Remember: changing the height also moves that box\'s top/bottom; '
                  'adjust its <pose> z if you want to keep the CoB where it is.')

    if a.frame:
        L, W, H = a.frame
        k = 0.5 * a.density * a.cd
        print(f'\nQuadratic drag estimates (Cd = {a.cd}), paste into the Hydrodynamics plugin:')
        # NUWave frame: x = right (sway), y = forward (surge), z = up (heave)
        print(f'  <xUabsU>{-k * L * H:.1f}</xUabsU>   <!-- sway,  A = {L} x {H} -->')
        print(f'  <yVabsV>{-k * W * H:.1f}</yVabsV>   <!-- surge, A = {W} x {H} -->')
        print(f'  <zWabsW>{-k * L * W:.1f}</zWabsW>   <!-- heave, A = {L} x {W} -->')
        print('  Rotational terms: start near -1.5 for a ROV this size, then calibrate with a spin test.')
    print()


if __name__ == '__main__':
    main()
