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

#ifndef PILOTARO_ACTUATORS__ACTUATORSNODE_HPP_
#define PILOTARO_ACTUATORS__ACTUATORSNODE_HPP_

#include <memory>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pilotaro_common/RobotState.hpp"
#include "pilotaro_core/ActuatorOutputBase.hpp"

namespace pilotaro
{

/**
 * @class ActuatorsNode
 * @brief A node that sends the commands stored in the robot state to the robot actuators.
 *
 * It is the last safety layer: whatever the controller says, the rotors are stopped in
 * DISARMED and at idle_rot_velocity in ARMED. While flying it sends the rotor speeds of
 * the controller. The output plugin ("output.plugin") turns them into messages for the
 * actuators: pilotaro_actuators/MotorSpeedOutput (one actuator_msgs/Actuators, Gazebo
 * multicopter motors) or pilotaro_actuators/ThrusterOutput (one std_msgs/Float64 per
 * thruster, Gazebo thrusters).
 */
class ActuatorsNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ActuatorsNode)

  /**
   * @brief Constructor for ActuatorsNode
   * @param NodeOptions to configurate node
   */
  explicit ActuatorsNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~ActuatorsNode();

  /**
   * @brief Load the output plugin and size the command for the vehicle
   * @param robot state object
   * @return bool success result
   */
  bool init(std::shared_ptr<RobotState> robot_state);

  /**
   * @brief Send the rotor speeds for the current mode
   * @param robot state object
   * @return bool success result
   */
  bool cycle(std::shared_ptr<RobotState> robot_state);

  /**
   * @brief Send zero speed to every rotor
   */
  void stop();

  /**
   * @brief Stop the rotors and unload the plugin
   */
  void release();

private:
  pluginlib::ClassLoader<ActuatorOutputBase> loader_;       ///< Must outlive the plugin

  std::shared_ptr<ActuatorOutputBase> output_;       ///< Loaded plugin

  double idle_rot_velocity_;       ///< Rotor speed when ARMED [rad/s]

  std::vector<double> command_;       ///< Preallocated command [rad/s]
};

}  // namespace pilotaro

#endif  // PILOTARO_ACTUATORS__ACTUATORSNODE_HPP_
