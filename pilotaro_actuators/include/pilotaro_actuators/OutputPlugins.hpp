//  Copyright 2026 Juan S. Cely G.

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      https://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef PILOTARO_ACTUATORS__OUTPUTPLUGINS_HPP_
#define PILOTARO_ACTUATORS__OUTPUTPLUGINS_HPP_

#include <string>
#include <vector>

#include "actuator_msgs/msg/actuators.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "pilotaro_core/ActuatorOutputBase.hpp"

namespace pilotaro_actuators
{

/**
 * @class MotorSpeedOutput
 * @brief All the rotor speeds in one actuator_msgs/Actuators ("velocity" field), as the
 * Gazebo MulticopterMotorModel expects.
 *
 * Parameters (prefix "output."): topic (string, "command/motor_speed").
 */
class MotorSpeedOutput : public pilotaro::ActuatorOutputBase
{
public:
  bool write(const std::vector<double> & rotor_speeds) override;

protected:
  bool on_initialize() override;

private:
  actuator_msgs::msg::Actuators msg_;       ///< Preallocated message
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_;
};

/**
 * @class ThrusterOutput
 * @brief One std_msgs/Float64 per thruster, as the Gazebo Thruster system expects.
 *
 * Parameters (prefix "output."):
 *  - topic_pattern (string, "thruster_{}/cmd_thrust"): topic of each thruster, "{}" is
 *    replaced by its index (relative to the robot namespace)
 *  - command (string, "thrust"): "thrust" sends motor_constant * w * |w| [N],
 *    "angular_velocity" sends w [rad/s]
 */
class ThrusterOutput : public pilotaro::ActuatorOutputBase
{
public:
  bool write(const std::vector<double> & rotor_speeds) override;

protected:
  bool on_initialize() override;

private:
  bool send_thrust_{true};       ///< Thrust (true) or angular velocity (false)
  std_msgs::msg::Float64 msg_;       ///< Reused message
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
};

}  // namespace pilotaro_actuators

#endif  // PILOTARO_ACTUATORS__OUTPUTPLUGINS_HPP_
