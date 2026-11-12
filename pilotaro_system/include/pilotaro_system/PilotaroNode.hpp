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

#ifndef PILOTARO_SYSTEM__PILOTARONODE_HPP_
#define PILOTARO_SYSTEM__PILOTARONODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "pilotaro_common/RobotState.hpp"

#include "pilotaro_states/StatesNode.hpp"
#include "pilotaro_estimators/EstimatorNode.hpp"
#include "pilotaro_actuators/ActuatorsNode.hpp"
#include "pilotaro_controllers/ControllersNode.hpp"


namespace pilotaro
{

/**
 * @struct PilotaroNodeInfo
 * @brief Contains information about a node managed by Pilotaro.
 */
struct PilotaroNodeInfo
{
  rclcpp::Node::SharedPtr node_ptr;       ///< Shared pointer to the managed node.
  rclcpp::CallbackGroup::SharedPtr cbg;       ///< Shared pointer to callbackgroup
};

/**
 * @class PilotaroNode
 * @brief A lifecycle node that manages the Pilotaro robot system.
 *
 * This is the only lifecycle node of the system. It owns the estimator, states,
 * controllers and actuators nodes, and shares with them the robot state. Each cycle runs
 * estimator -> states -> controllers -> actuators.
 *
 * Parameters: robot_name (default: the namespace), vehicle.* (physical properties, see
 * VehicleParams), frequency, spin_time and real_time.* (read by pilotaro_main).
 */
class PilotaroNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PilotaroNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor
   * @param Node options to configure the node
   */
  explicit PilotaroNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~PilotaroNode();

  /**
   * @brief Instructions on_configure state
   * @param state lifecycle
   * @return CallbackReturnT defined in this class
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Instructions on_activate state
   * @param state lifecycle
   * @return CallbackReturnT defined in this class
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Instructions on_deactive state
   * @param state lifecycle
   * @return CallbackReturnT defined in this class
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Instructions on_cleanup state
   * @param state lifecycle
   * @return CallbackReturnT defined in this class
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Instructions on_shutdown state
   * @param state lifecycle
   * @return CallbackReturnT defined in this class
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Instructions on_error state
   * @param state lifecycle
   * @return CallbackReturnT defined in this class
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief method to obtain updated information about nodes, overall Nodes names
   * @return map with name and info of the nodes managed by PilotaroNode
   */
  std::map<std::string, PilotaroNodeInfo> get_pilotaro_system_nodes();

  /**
   * @brief Initalize method, into are call init method of another nodes
   * @return bool success result
   */
  bool pilotaro_init();

  /**
   * @brief Cycle method, into are call cycle method of another nodes
   */
  void pilotaro_cycle();

  /**
   * @brief Shared robot state (read only)
   */
  std::shared_ptr<const RobotState> get_robot_state() const {return robot_state_;}

  /**
   * @brief preShutdown method is defined to configure lifecycle node before than shutdown
   */
  void preShutdown();

private:
  /**
   * @brief Unload the plugins and forget the robot state in the nodes
   */
  void pilotaro_release();

  /// Shared pointer to StatesNode instance
  StatesNode::SharedPtr states_node_ = StatesNode::make_shared();

  /// Shared pointer to EstimatorNode instance
  EstimatorNode::SharedPtr estimator_node_ = EstimatorNode::make_shared();

  /// Shared pointer to ActuatorsNode instance
  ActuatorsNode::SharedPtr actuators_node_ = ActuatorsNode::make_shared();

  /// Shared pointer to ControllersNode instance
  ControllersNode::SharedPtr controllers_node_ = ControllersNode::make_shared();

  /// Shared pointer pilotaro robot object
  std::shared_ptr<RobotState> robot_state_ = std::make_shared<RobotState>();

  std::string robot_name_;       ///< string name to call the robot
};

}  // namespace pilotaro

#endif  // PILOTARO_SYSTEM__PILOTARONODE_HPP_
