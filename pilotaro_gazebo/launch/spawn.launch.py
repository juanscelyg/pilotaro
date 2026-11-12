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
Add one vehicle of pilotaro_description to a running Gazebo world and fly it with pilotaro.

Everything of the robot is named after robot_name, so this launch can be included several
times with different names (multirobot): Gazebo model, topics (/<robot_name>/...), frames
(<robot_name>/...) and the namespace of pilotaro.

The bridge is built from the sensors of the URDF, so any vehicle described with the macros
of pilotaro_description/urdf/common.xacro works without changes here.
"""

from os.path import join
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro

# Height of base_link over the ground when the vehicle rests on it [m]. Underwater
# vehicles are dropped at the surface of the water (z = 0) and float
SPAWN_HEIGHT = {'x500': 0.24, 'bebop2': 0.065, 'bluerov2': 0.0}

# Gazebo sensor type -> (ROS type, Gazebo type) of its topic
SENSOR_TYPES = {
    'imu': ('sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
    'air_pressure': ('sensor_msgs/msg/FluidPressure', 'gz.msgs.FluidPressure'),
    'magnetometer': ('sensor_msgs/msg/MagneticField', 'gz.msgs.Magnetometer'),
    'navsat': ('sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat'),
    'camera': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
    'gpu_lidar': ('sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
}


def bridge_topics(robot, urdf):
    """(topic, ros type, gz type, direction) of every bridged topic of a robot."""
    description = ET.fromstring(urdf)
    topics = [(f'/{robot}/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', '[')]
    plugins = [p.get('filename') or '' for p in description.iter('plugin')]
    # Multirotor motors: one actuator_msgs/Actuators with every rotor speed
    if any('multicopter-motor-model' in p for p in plugins):
        topics.append((f'/{robot}/command/motor_speed', 'actuator_msgs/msg/Actuators',
                       'gz.msgs.Actuators', ']'))
    # Marine thrusters: one std_msgs/Float64 per thruster with its thrust, on
    # <namespace>/<topic>
    for plugin in description.iter('plugin'):
        if 'thruster-system' in (plugin.get('filename') or ''):
            namespace = plugin.findtext('namespace', robot).strip()
            topic = plugin.findtext('topic', '').strip()
            topics.append((f'/{namespace}/{topic}', 'std_msgs/msg/Float64', 'gz.msgs.Double', ']'))
    for sensor in description.iter('sensor'):
        kind = sensor.get('type')
        topic = sensor.findtext('topic')
        if kind not in SENSOR_TYPES or not topic:
            continue
        topics.append((topic.strip(), *SENSOR_TYPES[kind], '['))
        info = sensor.findtext('camera/camera_info_topic')
        if kind == 'camera' and info:
            topics.append((info.strip(), 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo', '['))
    return topics


# <xDotU> ... of the Hydrodynamics plugin -> <fluid_added_mass> (positive, 6x6 symmetric)
ADDED_MASS = {'xx': 'xDotU', 'yy': 'yDotV', 'zz': 'zDotW',
              'pp': 'kDotP', 'qq': 'mDotQ', 'rr': 'nDotR'}
FLUID_ADDED_MASS_ORDER = ['xx', 'xy', 'xz', 'xp', 'xq', 'xr', 'yy', 'yz', 'yp', 'yq', 'yr',
                          'zz', 'zp', 'zq', 'zr', 'pp', 'pq', 'pr', 'qq', 'qr', 'rr']


def gazebo_model(urdf):
    """
    SDF file to spawn a vehicle with hydrodynamics, None for the others.

    The URDF is converted to SDF (gz sdf, which keeps every Gazebo element) and the added
    mass of each Hydrodynamics plugin is moved to the <fluid_added_mass> of its link: DART
    integrates it implicitly, while the explicit added mass of the plugin is unstable when it
    is larger than the mass of the link. URDF has no element for it.
    """
    plugins = [p for p in ET.fromstring(urdf).iter('plugin')
               if 'hydrodynamics-system' in (p.get('filename') or '')]
    if not plugins:
        return None
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(urdf)
    sdf = ET.fromstring(subprocess.run(
        ['gz', 'sdf', '-p', f.name], check=True, capture_output=True, text=True).stdout)
    for plugin in sdf.iter('plugin'):
        if 'hydrodynamics-system' not in (plugin.get('filename') or ''):
            continue
        link = sdf.find(f".//link[@name='{plugin.findtext('link_name').strip()}']")
        inertial = link.find('inertial')
        added = inertial.find('fluid_added_mass')
        if added is None:
            added = ET.SubElement(inertial, 'fluid_added_mass')
        for key in FLUID_ADDED_MASS_ORDER:
            value = -float(plugin.findtext(ADDED_MASS[key], '0')) if key in ADDED_MASS else 0.0
            ET.SubElement(added, key).text = repr(value)
    with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
        f.write(ET.tostring(sdf, encoding='unicode'))
        return f.name


def launch_robot(context):
    robot = LaunchConfiguration('robot_name').perform(context).strip('/')
    model = LaunchConfiguration('model').perform(context)
    cameras = LaunchConfiguration('cameras').perform(context).lower() == 'true'
    z = LaunchConfiguration('z').perform(context) or str(SPAWN_HEIGHT.get(model, 0.3))

    urdf_file = join(
        get_package_share_directory('pilotaro_description'), 'urdf', f'{model}.urdf.xacro')
    urdf = xacro.process_file(
        urdf_file, mappings={'name': robot, 'cameras': str(cameras).lower()}).toxml()

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot,
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(urdf, value_type=str),
            'frame_prefix': f'{robot}/',
            'use_sim_time': True,
        }],
    )

    # Vehicles with hydrodynamics are spawned from their SDF, the others from the URDF
    model_file = gazebo_model(urdf)
    source = ['-file', model_file] if model_file else ['-topic', f'/{robot}/robot_description']
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name').perform(context),
            '-name', robot,
            *source,
            '-x', LaunchConfiguration('x').perform(context),
            '-y', LaunchConfiguration('y').perform(context),
            '-z', z,
            '-Y', LaunchConfiguration('yaw').perform(context),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot}_bridge',
        output='screen',
        arguments=[f'{topic}@{ros_type}{d}{gz_type}'
                   for topic, ros_type, gz_type, d in bridge_topics(robot, urdf)],
        parameters=[{'use_sim_time': True}],
    )

    pilotaro = IncludeLaunchDescription(
        join(get_package_share_directory('pilotaro'), 'launch', 'pilotaro.launch.py'),
        launch_arguments={
            'robot_name': robot,
            'vehicle': model,
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
        }.items(),
    )

    return [state_publisher, spawn, bridge, pilotaro]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'robot_name', default_value='x500',
        description='Name of the robot, unique in the simulation'))
    ld.add_action(DeclareLaunchArgument(
        'model', default_value='x500',
        description='Vehicle of pilotaro_description: ' + ', '.join(SPAWN_HEIGHT)))
    ld.add_action(DeclareLaunchArgument(
        'world_name', default_value='empty',
        description='Name of the running Gazebo world (<world name="...">)'))
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'z', default_value='',
        description='Spawn height, empty: resting on a ground at z = 0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'cameras', default_value='true', description='Add the cameras of the vehicle'))
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=join(
            get_package_share_directory('pilotaro'), 'params', 'pilotaro.params.yaml'),
        description='Main pilotaro parameters'))
    ld.add_action(OpaqueFunction(function=launch_robot))
    return ld
