#!/usr/bin/env python3
# Copyright 2026 Juan S. Cely G.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Update the "vehicle" block of a pilotaro params file from the URDF (or xacro) of the vehicle.

Each vehicle of pilotaro_description has its URDF in urdf/<vehicle>.urdf.xacro and its
parameters in config/<vehicle>.params.yaml, which pilotaro loads after the main params file.

From the URDF it takes:
  - mass: sum of the masses of every link
  - inertia: diagonal of the inertia tensor of the whole vehicle about its center of mass
  - rotor_x, rotor_y: position of each rotor joint relative to the center of mass
  - rotor_direction, motor_constant, moment_constant, max_rot_velocity: from the
    MulticopterMotorModel plugins (<gazebo> blocks), ordered by actuator number
gravity and min_rot_velocity are not in the URDF, so the values of the params file are kept.

Only the lines of the vehicle block change; the rest of the file (comments included) is
kept. If the params file does not exist, it is created.

Examples:
  update_vehicle_params.py                          # x500: urdf/x500.urdf.xacro ->
                                                    #   config/x500.params.yaml
  update_vehicle_params.py --vehicle bebop2         # urdf/bebop2.urdf.xacro -> config/...
  update_vehicle_params.py --urdf my_drone.urdf.xacro --params my.params.yaml
  update_vehicle_params.py --dry-run                # print the block, do not write
  update_vehicle_params.py --check                  # exit 1 if the file is out of date
"""

import argparse
import datetime
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

MOTOR_PLUGIN = 'multicopter-motor-model'
THRUSTER_PLUGIN = 'thruster-system'
KEPT_KEYS = ('gravity', 'min_rot_velocity')
DEFAULT_KEPT = {'gravity': 9.8, 'min_rot_velocity': 100.0}


def default_paths(vehicle):
    """
    URDF and params file of a vehicle.

    From the source tree if the script runs from it. From the install space the params file
    is not given: updating an installed file has no effect on the next build.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    src_urdf = os.path.join(here, '..', 'urdf', f'{vehicle}.urdf.xacro')
    src_params = os.path.join(here, '..', 'config', f'{vehicle}.params.yaml')
    if os.path.exists(src_urdf):
        return os.path.normpath(src_urdf), os.path.normpath(src_params)
    try:
        from ament_index_python.packages import get_package_share_directory
        urdf = os.path.join(
            get_package_share_directory('pilotaro_description'), 'urdf', f'{vehicle}.urdf.xacro')
        return urdf, None
    except Exception:  # noqa: B902
        return None, None


def load_urdf(path, mappings):
    if path.endswith('.xacro'):
        import xacro
        return xacro.process_file(path, mappings=mappings).toxml()
    with open(path) as f:
        return f.read()


def floats(text, count=3):
    values = [float(v) for v in (text or '').split()]
    return values if values else [0.0] * count


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin(element):
    """(rotation, translation) of the <origin> child of element (identity if missing)."""
    o = element.find('origin') if element is not None else None
    if o is None:
        return np.eye(3), np.zeros(3)
    return rpy_matrix(*floats(o.get('rpy'))), np.array(floats(o.get('xyz')))


def link_poses(robot):
    """Pose of every link in the root link frame, joints at zero position."""
    links = {link.get('name') for link in robot.findall('link')}
    parent_of = {}
    for joint in robot.findall('joint'):
        child = joint.find('child').get('link')
        parent_of[child] = (joint.find('parent').get('link'), *origin(joint))
    roots = [name for name in links if name not in parent_of]
    if len(roots) != 1:
        raise ValueError(f'The URDF must have one root link, found {roots}')
    poses = {roots[0]: (np.eye(3), np.zeros(3))}

    def pose(name):
        if name not in poses:
            parent, r, t = parent_of[name]
            pr, pt = pose(parent)
            poses[name] = (pr @ r, pr @ t + pt)
        return poses[name]

    for name in links:
        pose(name)
    return roots[0], poses


def mass_properties(robot, poses):
    """Total mass, center of mass and inertia tensor about it, in the root frame."""
    bodies = []
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        i = inertial.find('inertia')
        g = {k: float(i.get(k, 0.0)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
        tensor = np.array([
            [g['ixx'], g['ixy'], g['ixz']],
            [g['ixy'], g['iyy'], g['iyz']],
            [g['ixz'], g['iyz'], g['izz']]])
        lr, lt = poses[link.get('name')]
        ir, it = origin(inertial)
        rotation = lr @ ir
        bodies.append((mass, lr @ it + lt, rotation @ tensor @ rotation.T))
    total = sum(b[0] for b in bodies)
    if total <= 0.0:
        raise ValueError('The URDF has no mass')
    com = sum(m * c for m, c, _ in bodies) / total
    inertia = np.zeros((3, 3))
    for m, c, tensor in bodies:
        d = c - com
        inertia += tensor + m * (d @ d * np.eye(3) - np.outer(d, d))
    return total, com, inertia


def joint_axis(joint, poses):
    """Return the unit axis of a joint in the root frame (child link frame at zero)."""
    axis_element = joint.find('axis')
    axis = np.array(floats(axis_element.get('xyz')) if axis_element is not None else [1, 0, 0])
    axis = poses[joint.find('child').get('link')][0] @ axis
    return axis / np.linalg.norm(axis)


def motor_model(text):
    """Read the rotor of a MulticopterMotorModel plugin (propeller, forward only)."""
    direction = text('turningDirection').lower()
    if direction not in ('ccw', 'cw'):
        raise ValueError('turningDirection must be ccw or cw')
    number = text('actuator_number', False) or text('motorNumber', False)
    return {
        'number': None if number is None else int(number),
        'joint': text('jointName'),
        'direction': 1.0 if direction == 'ccw' else -1.0,
        'motor_constant': float(text('motorConstant')),
        'moment_constant': float(text('momentConstant')),
        'max_rot_velocity': float(text('maxRotVelocity')),
        'reversible': False,
    }


def thruster(text):
    """
    Rotor of a Thruster plugin (marine thruster).

    thrust = thrust_coefficient * fluid_density * propeller_diameter^4 * w * |w| along the
    joint axis; the sign of the coefficient is the spin direction. With velocity control
    Gazebo applies no reaction torque. Reversible if min_thrust_cmd < 0.
    """
    coefficient = float(text('thrust_coefficient'))
    density = float(text('fluid_density', False) or 1000.0)
    diameter = float(text('propeller_diameter', False) or 0.02)
    motor_constant = abs(coefficient) * density * diameter ** 4
    max_thrust = float(text('max_thrust_cmd', False) or 1000.0)
    min_thrust = float(text('min_thrust_cmd', False) or -1000.0)
    return {
        'number': None,
        'joint': text('joint_name'),
        'direction': 1.0 if coefficient >= 0.0 else -1.0,
        'motor_constant': motor_constant,
        'moment_constant': 0.0,
        'max_rot_velocity': math.sqrt(max(max_thrust, -min_thrust) / motor_constant),
        'reversible': min_thrust < 0.0,
    }


ACTUATOR_PLUGINS = {MOTOR_PLUGIN: motor_model, THRUSTER_PLUGIN: thruster}


def motors(robot, poses, com):
    """
    Rotors from the MulticopterMotorModel or Thruster plugins.

    Ordered by actuator number (motor models) or by order of appearance (thrusters).
    """
    joints = {j.get('name'): j for j in robot.findall('joint')}
    rotors = []
    for plugin in robot.iter('plugin'):
        filename = plugin.get('filename') or ''
        kind = next((k for k in ACTUATOR_PLUGINS if k in filename), None)
        if kind is None:
            continue

        def text(tag, required=True):
            element = plugin.find(tag)
            if element is None or element.text is None:
                if required:
                    raise ValueError(f'{filename} without <{tag}>')
                return None
            return element.text.strip()

        rotor = ACTUATOR_PLUGINS[kind](text)
        joint = joints.get(rotor['joint'])
        if joint is None:
            raise ValueError(f'{filename} joint {rotor["joint"]} is not in the URDF')
        position = poses[joint.find('child').get('link')][1] - com
        rotor.update({
            'kind': kind,
            'x': position[0],
            'y': position[1],
            'z': position[2],
            'axis': joint_axis(joint, poses),
        })
        if rotor['number'] is None:
            rotor['number'] = len(rotors)
        rotors.append(rotor)
    if len(rotors) < 4:
        raise ValueError(
            f'At least 4 MulticopterMotorModel or Thruster plugins are needed, '
            f'found {len(rotors)}')
    rotors.sort(key=lambda r: r['number'])
    for key in ('kind', 'motor_constant', 'moment_constant', 'max_rot_velocity', 'reversible'):
        values = {r[key] for r in rotors}
        if len(values) != 1:
            raise ValueError(f'Every rotor must have the same {key}, found {sorted(values)}')
    return rotors


def clean(value):
    """Round away the noise of the rotations (1e-17) and the negative zero."""
    return round(float(value), 9) + 0.0


def vehicle_from_urdf(xml):
    robot = ET.fromstring(xml)
    root, poses = link_poses(robot)
    mass, com, inertia = mass_properties(robot, poses)
    rotors = motors(robot, poses, com)
    off_diagonal = max(abs(inertia[0, 1]), abs(inertia[0, 2]), abs(inertia[1, 2]))
    vehicle = {
        'mass': mass,
        'inertia': [inertia[0, 0], inertia[1, 1], inertia[2, 2]],
        'rotor_x': [clean(r['x']) for r in rotors],
        'rotor_y': [clean(r['y']) for r in rotors],
        'rotor_z': [clean(r['z']) for r in rotors],
        'rotor_axis': [clean(a) for r in rotors for a in r['axis']],
        'rotor_direction': [r['direction'] for r in rotors],
        'motor_constant': rotors[0]['motor_constant'],
        'moment_constant': rotors[0]['moment_constant'],
        'max_rot_velocity': rotors[0]['max_rot_velocity'],
        'reversible': rotors[0]['reversible'],
    }
    info = {
        'root': root, 'com': com, 'off_diagonal': off_diagonal, 'rotors': rotors,
        'robot': robot.get('name'),
    }
    return vehicle, info


def number(value):
    """Float text that every YAML parser reads as a double (never as an integer)."""
    text = f'{value:.6g}'
    mantissa, _, exponent = text.partition('e')
    if '.' not in mantissa and 'inf' not in mantissa and 'nan' not in mantissa:
        mantissa += '.0'
    return mantissa + ('e' + exponent if exponent else '')


def render_block(vehicle, indent, source):
    def values(v):
        return '[' + ', '.join(number(x) for x in v) + ']'

    pad = ' ' * indent
    pad2 = ' ' * (indent + 2)
    min_comment = ('[rad/s] (0 for reversible thrusters)' if vehicle['reversible']
                   else '[rad/s] (kept, not in the URDF)')
    today = datetime.date.today().isoformat()
    rows = [
        ('mass', number(vehicle['mass']), '[kg]'),
        ('gravity', number(vehicle['gravity']), '[m/s^2] (kept, not in the URDF)'),
        ('inertia', values(vehicle['inertia']), '[kg m^2] about the center of mass'),
        ('rotor_x', values(vehicle['rotor_x']), '[m] from the center of mass'),
        ('rotor_y', values(vehicle['rotor_y']), '[m] from the center of mass'),
        ('rotor_z', values(vehicle['rotor_z']), '[m] from the center of mass'),
        ('rotor_axis', values(vehicle['rotor_axis']), 'thrust direction (x, y, z) per rotor'),
        ('rotor_direction', values(vehicle['rotor_direction']), '1 = ccw, -1 = cw'),
        ('motor_constant', number(vehicle['motor_constant']), 'thrust = k * w * |w|'),
        ('moment_constant', number(vehicle['moment_constant']), 'drag torque = c * thrust'),
        ('min_rot_velocity', number(vehicle['min_rot_velocity']), min_comment),
        ('max_rot_velocity', number(vehicle['max_rot_velocity']), '[rad/s]'),
        ('reversible', 'true' if vehicle['reversible'] else 'false',
         'thrusters spin (and push) both ways'),
    ]
    # Comments aligned in a column; a long row gets its comment on the line above
    long_row = 56
    width = max(len(f'{k}: {v}') for k, v, _ in rows if len(f'{k}: {v}') <= long_row) + 2
    lines = [f'{pad}# Generated from {source} by update_vehicle_params.py ({today})\n',
             f'{pad}vehicle:\n']
    for k, v, c in rows:
        row = f'{k}: {v}'
        if len(row) > long_row:
            lines += [f'{pad2}# {c}\n', f'{pad2}{row}\n']
        else:
            lines.append(f'{pad2}{row.ljust(width)}# {c}\n')
    return lines


def indentation(line):
    return len(line) - len(line.lstrip(' '))


def find_block(lines, node):
    """(start, end, indent) of the vehicle block of node, or insertion point if missing."""
    node_re = re.compile(rf'^\S*{re.escape(node)}:\s*(#.*)?$')
    start_node = next((i for i, line in enumerate(lines) if node_re.match(line)), None)
    if start_node is None:
        raise ValueError(f'Node "{node}" not found in the params file')
    end_node = next(
        (i for i in range(start_node + 1, len(lines))
         if lines[i].strip() and indentation(lines[i]) == 0 and not lines[i].startswith('#')),
        len(lines))
    vehicle_re = re.compile(r'^(\s+)vehicle:\s*(#.*)?$')
    for i in range(start_node + 1, end_node):
        match = vehicle_re.match(lines[i])
        if match:
            indent = len(match.group(1))
            end = i + 1
            while end < end_node and (
                    not lines[end].strip() or indentation(lines[end]) > indent):
                end += 1
            while end > i + 1 and not lines[end - 1].strip():
                end -= 1  # keep the blank lines after the block
            start = i
            if i > 0 and lines[i - 1].strip().startswith('# Generated from'):
                start = i - 1
            return start, end, indent
    params_re = re.compile(r'^(\s+)ros__parameters:\s*(#.*)?$')
    for i in range(start_node + 1, end_node):
        match = params_re.match(lines[i])
        if match:
            return i + 1, i + 1, len(match.group(1)) + 2
    raise ValueError(f'"ros__parameters" not found for node "{node}"')


def kept_values(text, node):
    data = yaml.safe_load(text) or {}
    for key, value in data.items():
        if str(key).rstrip(':').endswith(node):
            vehicle = ((value or {}).get('ros__parameters') or {}).get('vehicle') or {}
            return {k: float(vehicle.get(k, DEFAULT_KEPT[k])) for k in KEPT_KEYS}
    return dict(DEFAULT_KEPT)


LICENSE_HEADER = (
    '# Copyright 2026 Juan S. Cely G.\n'
    '#\n'
    '# Licensed under the Apache License, Version 2.0 (the "License");\n'
    '# you may not use this file except in compliance with the License.\n'
    '# You may obtain a copy of the License at\n'
    '#\n'
    '#     https://www.apache.org/licenses/LICENSE-2.0\n'
    '#\n'
    '# Unless required by applicable law or agreed to in writing, software\n'
    '# distributed under the License is distributed on an "AS IS" BASIS,\n'
    '# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n'
    '# See the License for the specific language governing permissions and\n'
    '# limitations under the License.\n'
    '\n')


def new_params_file(vehicle, node):
    return (f'{LICENSE_HEADER}'
            f'# Parameters of the {vehicle} for pilotaro, loaded after pilotaro.params.yaml.\n'
            f'# The vehicle block is generated from urdf/{vehicle}.urdf.xacro by\n'
            f'# scripts/update_vehicle_params.py. Vehicle specific tuning can be added below.\n'
            f'\n/**/{node}:\n  ros__parameters:\n')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--vehicle', default='x500',
                        help='vehicle of pilotaro_description, sets the default --urdf and '
                             '--params (default x500)')
    parser.add_argument('--urdf', help='URDF or xacro of the vehicle')
    parser.add_argument('--params', help='params file to update')
    parser.add_argument('--node', default='pilotaro_node',
                        help='node whose vehicle block is updated (default pilotaro_node)')
    parser.add_argument('--xacro-arg', action='append', default=[], metavar='NAME:=VALUE',
                        help='xacro argument, can be repeated')
    parser.add_argument('--dry-run', action='store_true', help='print, do not write')
    parser.add_argument('--check', action='store_true',
                        help='do not write; exit 1 if the params file is out of date')
    args = parser.parse_args()
    default_urdf, default_params = default_paths(args.vehicle)
    args.urdf = args.urdf or default_urdf
    args.params = args.params or default_params
    if not args.urdf:
        parser.error('--urdf is required')
    if not args.params and not args.dry_run:
        parser.error('--params is required')

    mappings = {}
    for item in args.xacro_arg:
        name, sep, value = item.partition(':=')
        if not sep:
            parser.error(f'--xacro-arg must be NAME:=VALUE, got {item}')
        mappings[name] = value

    vehicle, info = vehicle_from_urdf(load_urdf(args.urdf, mappings))
    print(f'Vehicle "{info["robot"]}" from {args.urdf}')
    print(f'  mass {vehicle["mass"]:.4f} kg, center of mass {np.round(info["com"], 4)} m '
          f'in {info["root"]}')
    print(f'  inertia {np.round(vehicle["inertia"], 5)} kg m^2'
          f' (largest product of inertia {info["off_diagonal"]:.2e})')
    for r in info['rotors']:
        axis = ' '.join(f'{a:+.3f}' for a in r['axis'])
        spin = 'ccw' if r['direction'] > 0 else 'cw'
        print(f'  rotor {r["number"]} ({r["joint"]}): x {r["x"]:+.4f} y {r["y"]:+.4f} '
              f'z {r["z"]:+.4f}, axis [{axis}], {spin}')

    text = ''
    if args.params:
        if os.path.exists(args.params):
            with open(args.params) as f:
                text = f.read()
        else:
            text = new_params_file(info['robot'], args.node)
    vehicle.update(kept_values(text, args.node) if text else DEFAULT_KEPT)
    if vehicle['reversible']:
        vehicle['min_rot_velocity'] = 0.0
    source = os.path.basename(args.urdf)

    if not args.params:
        sys.stdout.write(''.join(render_block(vehicle, 4, source)))
        return 0

    lines = text.splitlines(keepends=True)
    start, end, indent = find_block(lines, args.node)
    block = render_block(vehicle, indent, source)
    new_lines = lines[:start] + block + lines[end:]

    # Compare ignoring the date of the generated comment
    def strip(ls):
        return [ln for ln in ls if '# Generated from' not in ln]

    changed = strip(new_lines) != strip(lines)
    if args.dry_run or args.check:
        sys.stdout.write(''.join(block))
        if args.check:
            print(f'{args.params} is {"out of date" if changed else "up to date"}')
            return 1 if changed else 0
        return 0
    if not changed:
        print(f'{args.params} is already up to date')
        return 0
    os.makedirs(os.path.dirname(os.path.abspath(args.params)), exist_ok=True)
    with open(args.params, 'w') as f:
        f.write(''.join(new_lines))
    print(f'Updated the vehicle block of {args.node} in {args.params}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, ET.ParseError) as e:
        print(f'Error: {e}', file=sys.stderr)
        sys.exit(1)
