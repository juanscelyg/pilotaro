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
Gazebo with a world, the clock bridge and one or more vehicles flown by pilotaro.

robots is a comma separated list of "name" or "name:model" (model defaults to the model
argument), e.g. robots:=uav1,uav2 or robots:=big:x500,small:bebop2. Robot i is spawned at
(x + i * spacing, y). Other robots can be added later with spawn.launch.py.
"""

from os.path import basename, join, splitext

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_sim(context):
    pkg = get_package_share_directory('pilotaro_gazebo')
    world = LaunchConfiguration('world').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    # A world of this package by name: world:=underwater
    if not world.endswith('.sdf') and '/' not in world:
        world_name = world_name or world
        world = join(pkg, 'worlds', world + '.sdf')
    world_name = world_name or splitext(basename(world))[0]
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args = '-r ' + ('-s --headless-rendering ' if headless else '') + world

    actions = [IncludeLaunchDescription(
        join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )]

    # One clock bridge for the whole simulation
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    ))

    default_model = LaunchConfiguration('model').perform(context)
    entries = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',')]
    x = float(LaunchConfiguration('x').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))
    for i, entry in enumerate(e for e in entries if e):
        robot, _, model = entry.partition(':')
        actions.append(IncludeLaunchDescription(
            join(pkg, 'launch', 'spawn.launch.py'),
            launch_arguments={
                'robot_name': robot,
                'model': model or default_model,
                'world_name': world_name,
                'x': str(x + i * spacing),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'cameras': LaunchConfiguration('cameras'),
                'params_file': LaunchConfiguration('params_file'),
            }.items(),
        ))
    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='empty',
        description='Gazebo world: a world of pilotaro_gazebo (empty, underwater) or a path'))
    ld.add_action(DeclareLaunchArgument(
        'world_name', default_value='',
        description='<world name> of the world file, empty: the name of the file'))
    ld.add_action(DeclareLaunchArgument(
        'headless', default_value='false', description='Run Gazebo server only, without GUI'))
    ld.add_action(DeclareLaunchArgument(
        'robots', default_value='x500',
        description='Comma separated "name" or "name:model"'))
    ld.add_action(DeclareLaunchArgument(
        'model', default_value='x500',
        description='Vehicle of the robots without model: x500, bebop2, bluerov2'))
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'spacing', default_value='2.0', description='Distance in x between robots [m]'))
    ld.add_action(DeclareLaunchArgument(
        'cameras', default_value='true', description='Add the cameras of the vehicles'))
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=join(
            get_package_share_directory('pilotaro'), 'params', 'pilotaro.params.yaml'),
        description='Main pilotaro parameters'))
    ld.add_action(OpaqueFunction(function=launch_sim))
    return ld
