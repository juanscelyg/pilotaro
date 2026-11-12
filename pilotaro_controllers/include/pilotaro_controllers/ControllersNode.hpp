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

#ifndef PILOTARO_CONTROLLERS__CONTROLLERSNODE_HPP_
#define PILOTARO_CONTROLLERS__CONTROLLERSNODE_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pilotaro_common/RobotState.hpp"
#include "pilotaro_core/ControllerBase.hpp"

namespace pilotaro
{

/**
 * @class ControllersNode
 * @brief A node that hosts the controller plugin and computes the rotor speeds from the
 * robot state.
 *
 * The controller runs only while flying and once per new estimate, with the time step
 * taken from the estimate stamps (clamped to [min_dt, max_dt]).
 *
 * Parameters: "controller.plugin" (pluginlib class, e.g.
 * "pilotaro_controllers/CascadeController"), "min_dt", "max_dt" and the parameters of
 * the plugin under "controller.".
 */
class ControllersNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ControllersNode)

  /**
   * @brief Constructor for ControllersNode
   * @param NodeOptions to configurate node
   */
  explicit ControllersNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~ControllersNode();

  /**
   * @brief Load and initialize the controller plugin
   * @param robot state object, with the vehicle parameters already set
   * @return bool success result
   */
  bool init(std::shared_ptr<RobotState> robot_state);

  /**
   * @brief Run the controller on a new estimate while flying
   * @param robot state object
   * @return bool success result
   */
  bool cycle(std::shared_ptr<RobotState> robot_state);

  /**
   * @brief Unload the plugin
   */
  void release();

private:
  pluginlib::ClassLoader<ControllerBase> loader_;       ///< Must outlive the plugin

  std::shared_ptr<ControllerBase> controller_;       ///< Loaded plugin

  std::vector<double> rotor_speeds_;       ///< Buffer for the plugin output

  uint64_t last_seq_{0};       ///< Estimate used in the last control step

  double last_stamp_{0.0};       ///< Stamp of that estimate [s]

  bool has_last_{false};       ///< A control step was run since the vehicle started flying

  double min_dt_;       ///< Lower bound of the time step [s]

  double max_dt_;       ///< Upper bound of the time step [s]
};

}  // namespace pilotaro

#endif  // PILOTARO_CONTROLLERS__CONTROLLERSNODE_HPP_
