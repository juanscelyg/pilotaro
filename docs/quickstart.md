# Quick start, in detail

Build pilotaro, start a simulation with any of its vehicles and fly it with the services and setpoints of [its ROS 2 interface](interfaces.md).

```bash
cd ~/ros_ws   # your colcon workspace, with pilotaro cloned in src/
rosdep update
rosdep install -y --ignore-src --from-paths src
colcon build --packages-up-to pilotaro pilotaro_gazebo --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Gazebo (GUI) + x500 + pilotaro. headless:=true without GUI, cameras:=false without cameras
ros2 launch pilotaro_gazebo sim.launch.py

# The same with a Parrot Bebop 2
ros2 launch pilotaro_gazebo sim.launch.py model:=bebop2 robots:=bebop2

# A BlueROV2 Heavy in the underwater world (floats at the surface until armed)
ros2 launch pilotaro_gazebo sim.launch.py world:=underwater robots:=rov:bluerov2_heavy
```

Build in `Release`: without a build type colcon compiles without optimization, and the
control loop and the EKF use several times more CPU.

In another terminal (the namespace is the robot name, `x500` by default):

```bash
ros2 service call /x500/arming  std_srvs/srv/Trigger
ros2 service call /x500/takeoff std_srvs/srv/Trigger     # climbs 2 m and switches to LOITER

# offboard: first publish setpoints (>= 2 Hz), then request the mode
ros2 topic pub -r 10 /x500/offboard/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 2.0, z: 3.0}, orientation: {w: 1.0}}}"
ros2 service call /x500/offboard std_srvs/srv/Trigger

ros2 service call /x500/landing std_srvs/srv/Trigger     # lands and disarms
ros2 topic echo /x500/mode
```

## Multi-robot: platform name

Each pilotaro instance is a process with a platform name, `robot_name`. That name is used
everywhere:

- **namespace** of the five nodes in the process: `/<robot_name>/pilotaro_node`,
  `/<robot_name>/states_node`… and therefore of all their topics and services;
- **robot name** inside pilotaro (`RobotState::get_name()`). If not given, it is taken from
  the namespace;
- in simulation, **Gazebo model name**, prefix of its topics (`/<robot_name>/imu`…),
  namespace of the motors and prefix of the frames (`<robot_name>/base_link`).

```bash
# Two drones in the same simulation
ros2 launch pilotaro_gazebo sim.launch.py robots:=uav1,uav2 spacing:=2.0

# Different models: "name:model"
ros2 launch pilotaro_gazebo sim.launch.py robots:=big:x500,small:bebop2 spacing:=3.0

# Or add one more to a running simulation
ros2 launch pilotaro_gazebo spawn.launch.py robot_name:=uav3 model:=bebop2 x:=4.0

ros2 service call /uav2/arming std_srvs/srv/Trigger
```

The parameter files use `/**/<node>:`, so they work for every robot. Each robot also loads
those of its vehicle (see [Parameters](parameters.md)); if it needs its own values, pass it a
different `params_file` or `vehicle_params_file`.

## Launch files

| Launch | What it does |
|---|---|
| `pilotaro pilotaro.launch.py` | pilotaro only (`robot_name`, `vehicle`, `params_file`, `vehicle_params_file`, `use_sim_time`). This is the one used on the real robot. |
| `pilotaro_gazebo spawn.launch.py` | Adds a vehicle to a running Gazebo: `robot_state_publisher`, spawn, bridge and pilotaro (`robot_name`, `model`, `world_name`, `x`, `y`, `z`, `yaw`, `cameras`, `params_file`). The bridge is generated from the sensors in the URDF. |
| `pilotaro_gazebo sim.launch.py` | Gazebo with `worlds/empty.sdf`, `/clock` bridge and one or more drones (`robots`, `model`, `spacing`, `headless`…). |
| `pilotaro_gazebo disaster.launch.py` | `disaster_nathan_benderson_park_world` world (`disaster_world` package, optional: not a declared dependency, clone it into `src/` to use this launch) with one vehicle at (-120, 950) (`model`). |

[Back to the README](../README.md)
