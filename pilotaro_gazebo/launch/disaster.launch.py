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

"""A vehicle flown by pilotaro in disaster_nathan_benderson_park_world (package disaster_world)."""

from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD = 'disaster_nathan_benderson_park_world'


def launch_world(context):
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    return [IncludeLaunchDescription(
        join(get_package_share_directory('disaster_world'), 'launch', 'disaster_world.launch.py'),
        launch_arguments={
            'world': WORLD + '.sdf',
            'gz_args': '-r -s --headless-rendering' if headless else '-r',
        }.items(),
    )]


def generate_launch_description():
    # Start pose suggested in the disaster_world README (x = -120, y = 950). The pavement
    # is at z ~ 4.6, so the drone is dropped from a few centimetres above it (x500; lower it
    # for the bebop2).
    robot = IncludeLaunchDescription(
        join(get_package_share_directory('pilotaro_gazebo'), 'launch', 'spawn.launch.py'),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'model': LaunchConfiguration('model'),
            'world_name': WORLD,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'cameras': LaunchConfiguration('cameras'),
        }.items(),
    )
    clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'headless', default_value='false', description='Run Gazebo server only, without GUI'))
    ld.add_action(DeclareLaunchArgument('robot_name', default_value='x500'))
    ld.add_action(DeclareLaunchArgument(
        'model', default_value='x500', description='Vehicle: x500, bebop2'))
    ld.add_action(DeclareLaunchArgument('x', default_value='-120.0'))
    ld.add_action(DeclareLaunchArgument('y', default_value='950.0'))
    ld.add_action(DeclareLaunchArgument('z', default_value='4.9'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('cameras', default_value='true'))
    ld.add_action(OpaqueFunction(function=launch_world))
    ld.add_action(clock)
    ld.add_action(robot)
    return ld
