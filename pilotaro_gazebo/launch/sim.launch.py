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

import importlib.util
from os import environ, pathsep
from os.path import basename, join, splitext
import shlex
import shutil

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def gazebo_server(gz_args):
    """Gazebo server as ros_gz_sim gz_sim.launch.py starts it, but without a shell.

    That launch runs 'sh -c "ruby gz sim ..."' and ruby replaces itself by the server: the
    SIGINT of the shutdown reaches only the shell, and the server (gz-sim-main) was left
    running, so the next simulation in the same partition found it (two servers of the
    same world).
    """
    spec = importlib.util.spec_from_file_location(
        'ros_gz_sim_gz_sim_launch',
        join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model_paths, plugin_paths = module.GazeboRosPaths.get_paths()
    env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': pathsep.join([
            environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''), environ.get('LD_LIBRARY_PATH', ''),
            plugin_paths]),
        'GZ_SIM_RESOURCE_PATH': pathsep.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''), model_paths]),
    }
    return ExecuteProcess(
        cmd=['ruby', shutil.which('gz'), 'sim', *shlex.split(gz_args), '--force-version', '11'],
        name='gazebo', output='screen', additional_env=env, on_exit=Shutdown())

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

    actions = [gazebo_server(gz_args)]

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
        description='Vehicle of the robots without model: x500, bebop2, bluerov2_heavy'))
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
