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

#ifndef PILOTARO_STATES__STATESNODE_HPP_
#define PILOTARO_STATES__STATESNODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "pilotaro_common/RobotState.hpp"
#include "pilotaro_states/FlightModeMachine.hpp"

namespace pilotaro
{

/**
 * @class StatesNode
 * @brief A node that manages the operational states (modes and transitions) of the robot.
 *
 * Services: arming, takeoff, landing, loiter, offboard (std_srvs/srv/Trigger).
 * Input: offboard/pose (geometry_msgs/PoseStamped), offboard/twist (geometry_msgs/Twist).
 * Output: mode (std_msgs/String, transient local).
 */
class StatesNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(StatesNode)
  using Trigger = std_srvs::srv::Trigger;

  /**
   * @brief Constructor for StatesNode
   * @param NodeOptions to configurate node
   */
  explicit StatesNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~StatesNode();

  /**
   * @brief Store the robot state and start in DISARMED
   * @param robot state object
   * @return bool success result
   */
  bool init(std::shared_ptr<RobotState> robot_state);

  /**
   * @brief Run the state machine
   * @param robot state object
   * @return bool success result
   */
  bool cycle(std::shared_ptr<RobotState> robot_state);

  /**
   * @brief Stop accepting commands
   */
  void release();

private:
  /**
   * @brief Run a command on the state machine and answer the service
   */
  void command(
    const std::string & name, std::shared_ptr<Trigger::Response> response,
    const std::function<CommandResult(RobotState &)> & action);

  /**
   * @brief Store an offboard position setpoint
   */
  void pose_setpoint_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  /**
   * @brief Store an offboard velocity setpoint
   */
  void twist_setpoint_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg);

  /**
   * @brief Log and publish a mode change
   */
  void on_mode_change(FlightMode from, FlightMode to, const std::string & reason);

  FlightModeMachine machine_;       ///< Flight modes
  std::shared_ptr<RobotState> robot_state_;       ///< Set by init()
  std::string offboard_frame_id_;       ///< Expected frame of offboard/pose, empty accepts any

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sp_sub_;
  rclcpp::Service<Trigger>::SharedPtr arming_srv_;
  rclcpp::Service<Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<Trigger>::SharedPtr landing_srv_;
  rclcpp::Service<Trigger>::SharedPtr loiter_srv_;
  rclcpp::Service<Trigger>::SharedPtr offboard_srv_;
};

}  // namespace pilotaro

#endif  // PILOTARO_STATES__STATESNODE_HPP_
