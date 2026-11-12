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

from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_pilotaro(context):
    vehicle_file = LaunchConfiguration('vehicle_params_file').perform(context)
    if not vehicle_file:
        vehicle = LaunchConfiguration('vehicle').perform(context)
        vehicle_file = join(
            get_package_share_directory('pilotaro_description'), 'config',
            f'{vehicle}.params.yaml')

    # One process per robot. Every node of the process (pilotaro_node, estimator_node,
    # states_node, controllers_node, actuators_node) runs in the namespace of the robot,
    # so several robots can share a ROS domain. Do not set "name": it would rename all
    # the nodes of the process to the same name.
    return [Node(
        package='pilotaro_system',
        executable='pilotaro_main',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            vehicle_file,
            {
                'robot_name': LaunchConfiguration('robot_name'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'simulation': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            },
        ],
    )]


def generate_launch_description():
    params_file = join(get_package_share_directory('pilotaro'),
                       'params', 'pilotaro.params.yaml')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'robot_name', default_value='x500',
        description='Name of the platform: namespace of its topics and services, unique in a '
                    'multirobot system'))
    ld.add_action(DeclareLaunchArgument(
        'vehicle', default_value='x500',
        description='Vehicle of pilotaro_description (x500, bebop2): loads '
                    'config/<vehicle>.params.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'vehicle_params_file', default_value='',
        description='Vehicle parameters, instead of the ones of "vehicle"'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=params_file, description='Main pilotaro parameters'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use the simulation clock'))
    ld.add_action(OpaqueFunction(function=launch_pilotaro))
    return ld
