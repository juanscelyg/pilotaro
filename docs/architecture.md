# Architecture and plugins

`pilotaro_main` runs one process per robot with five nodes (`pilotaro_node`, `estimator_node`, `states_node`, `controllers_node`, `actuators_node`) that share a robot state. Every cycle it runs estimator → states → controller → actuators in a single real-time thread ([Real time](realtime.md)). The estimator, the controller and the actuator output are pluginlib plugins.

## Plugins

Plugins are selected and configured in `params/pilotaro.params.yaml`, and the vehicle file
can change them (the BlueROV2 Heavy changes the controller and the output). The node that loads
them reads `<plugin_name>.plugin`, and the plugin reads its parameters with the same prefix:

```yaml
/**/estimator_node:
  ros__parameters:
    estimator:
      plugin: "pilotaro_estimators/OdometryEstimator"
      topic: "odometry"
      twist_in_body_frame: true

/**/controllers_node:
  ros__parameters:
    controller:
      plugin: "pilotaro_controllers/CascadeController"
      position_p: [1.0, 1.0, 1.5]
      # ...

/**/actuators_node:
  ros__parameters:
    output:
      plugin: "pilotaro_actuators/MotorSpeedOutput"
      topic: "command/motor_speed"
```

| Node | Interface | Plugins |
|---|---|---|
| `estimator_node` | `pilotaro::StateEstimatorBase` | `pilotaro_estimators/OdometryEstimator`, `pilotaro_estimators/InsEkfEstimator` |
| `controllers_node` | `pilotaro::ControllerBase` | `pilotaro_controllers/CascadeController`, `pilotaro_controllers/UnderwaterController` |
| `actuators_node` | `pilotaro::ActuatorOutputBase` | `pilotaro_actuators/MotorSpeedOutput`, `pilotaro_actuators/ThrusterOutput` |

The plugins of each kind are described in [State estimators](estimators.md) and [Controllers](controllers.md).

## Actuator output

`ActuatorsNode` decides the speed of each rotor every cycle: 0 in DISARMED,
`idle_rot_velocity` in ARMED and the controller output in flight. The plugin sends it:

- **`MotorSpeedOutput`:** one `actuator_msgs/Actuators` with all speeds (`topic` parameter).
  This is the format of Gazebo's `MulticopterMotorModel`.
- **`ThrusterOutput`:** one `std_msgs/Float64` per thruster, on `topic_pattern`
  (`thruster_{}/cmd_thrust`). With `command: thrust` it sends the thrust in N
  (`motor_constant·w·|w|`); with `angular_velocity`, the speed. This is the format of
  Gazebo's `Thruster` system.

## Writing a new plugin

An estimator derives from `pilotaro::StateEstimatorBase`; a controller, from
`pilotaro::ControllerBase`. For example, an estimator fusing IMU and GPS:

```cpp
#include "pilotaro_core/StateEstimatorBase.hpp"

namespace my_estimators
{
class ImuGpsEstimator : public pilotaro::StateEstimatorBase
{
public:
  // Called every cycle. true if `estimate` holds a new estimate
  bool update(pilotaro::VehicleState & estimate) override;
  void reset() override;

protected:
  // Read parameters (prefixed with "estimator.") and create subscriptions
  bool on_initialize() override
  {
    gps_noise_ = declare_parameter("gps_noise", 0.5);
    imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "imu"), rclcpp::SensorDataQoS(), ...);
    // get_vehicle(): mass, inertia, rotors...
    return true;
  }
};
}  // namespace my_estimators

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_estimators::ImuGpsEstimator, pilotaro::StateEstimatorBase)
```

```xml
<!-- my_estimators_plugins.xml -->
<library path="my_estimators">
  <class name="my_estimators/ImuGpsEstimator" type="my_estimators::ImuGpsEstimator"
         base_class_type="pilotaro::StateEstimatorBase">
    <description>EKF with IMU and GPS</description>
  </class>
</library>
```

```cmake
pluginlib_export_plugin_description_file(pilotaro_core my_estimators_plugins.xml)
```

Then just set `estimator.plugin: "my_estimators/ImuGpsEstimator"` in the parameter file.
Callbacks and `update()` run on the real-time thread: avoid blocking calls and memory
allocations inside `update()`.

[Back to the README](../README.md)
