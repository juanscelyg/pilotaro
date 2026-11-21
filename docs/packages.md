# Packages

The repository is a set of ament packages built with colcon.

| Package | Contents |
|---|---|
| `pilotaro` | Metapackage: `launch/pilotaro.launch.py` and `params/pilotaro.params.yaml`. |
| `pilotaro_common` | ROS-free types: `RobotState`, `VehicleState`, `ControlSetpoint`, `VehicleParams`, `FlightMode`. |
| `pilotaro_core` | Plugin interfaces: `StateEstimatorBase`, `ControllerBase`, `ActuatorOutputBase`, and parameter loading. |
| `pilotaro_estimators` | `EstimatorNode` (loads the plugin, publishes the estimate) and the `OdometryEstimator` and `InsEkfEstimator` (IMU-driven EKF) plugins. |
| `pilotaro_controllers` | `ControllersNode` (loads the plugin) and the `CascadeController` (multirotors) and `UnderwaterController` (vectored thrusters) plugins. |
| `pilotaro_states` | `StatesNode`: flight modes, services, offboard setpoints. The logic lives in `FlightModeMachine` (ROS-free, with tests). |
| `pilotaro_actuators` | `ActuatorsNode` (per-mode policy, loads the plugin) and the `MotorSpeedOutput` and `ThrusterOutput` output plugins. |
| `pilotaro_system` | `PilotaroNode` (lifecycle) and `pilotaro_main`, the real-time loop. |
| `pilotaro_description` | Vehicles: `urdf/<vehicle>.urdf.xacro`, `config/<vehicle>.params.yaml`, `meshes/` (BlueROV2 Heavy) and `scripts/update_vehicle_params.py`. |
| `pilotaro_gazebo` | Empty and underwater worlds, simulation launches (one or several vehicles of any model, disaster world), `scripts/flight_check.py`, `scripts/mission_check.py` and `scripts/failsafe_check.py`. |

[Back to the README](../README.md)
