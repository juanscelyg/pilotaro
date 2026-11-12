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

#ifndef PILOTARO_ESTIMATORS__ESTIMATORNODE_HPP_
#define PILOTARO_ESTIMATORS__ESTIMATORNODE_HPP_

#include <memory>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pilotaro_common/RobotState.hpp"
#include "pilotaro_core/StateEstimatorBase.hpp"

namespace pilotaro
{

/**
 * @class EstimatorNode
 * @brief A node that hosts the state estimator plugin and writes its estimates in the
 * robot state.
 *
 * Parameters: "estimator.plugin" (pluginlib class, e.g.
 * "pilotaro_estimators/OdometryEstimator") and the parameters of the plugin under
 * "estimator.".
 */
class EstimatorNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EstimatorNode)

  /**
   * @brief Constructor for EstimatorNode
   * @param NodeOptions to configurate node
   */
  explicit EstimatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~EstimatorNode();

  /**
   * @brief Load and initialize the estimator plugin
   * @param robot state object, with the vehicle parameters already set
   * @return bool success result
   */
  bool init(std::shared_ptr<RobotState> robot_state);

  /**
   * @brief Store the newest estimate, if any, in the robot state
   * @param robot state object
   * @return bool success result
   */
  bool cycle(std::shared_ptr<RobotState> robot_state);

  /**
   * @brief Unload the plugin
   */
  void release();

private:
  pluginlib::ClassLoader<StateEstimatorBase> loader_;       ///< Must outlive the plugin

  std::shared_ptr<StateEstimatorBase> estimator_;       ///< Loaded plugin

  VehicleState estimate_;       ///< Buffer for the plugin output
};

}  // namespace pilotaro

#endif  // PILOTARO_ESTIMATORS__ESTIMATORNODE_HPP_
