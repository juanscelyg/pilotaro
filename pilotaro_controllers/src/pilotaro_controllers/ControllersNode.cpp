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

#include <algorithm>
#include <string>

#include "pilotaro_controllers/ControllersNode.hpp"

namespace pilotaro
{

ControllersNode::ControllersNode(
  const rclcpp::NodeOptions & options)
: Node("controllers_node", options),
  loader_("pilotaro_core", "pilotaro::ControllerBase")
{
  declare_parameter("controller.plugin", std::string("pilotaro_controllers/CascadeController"));
  min_dt_ = declare_parameter("min_dt", 1e-4);
  max_dt_ = declare_parameter("max_dt", 0.05);
}

ControllersNode::~ControllersNode()
{
  release();
}

bool
ControllersNode::init(std::shared_ptr<RobotState> robot_state)
{
  const auto plugin = get_parameter("controller.plugin").as_string();
  try {
    controller_ = loader_.createSharedInstance(plugin);
    if (!controller_->initialize(shared_from_this(), "controller", robot_state->get_vehicle())) {
      RCLCPP_ERROR(get_logger(), "Controller [%s] failed to initialize", plugin.c_str());
      controller_.reset();
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Unable to load controller [%s]: %s", plugin.c_str(), e.what());
    controller_.reset();
    return false;
  }
  rotor_speeds_.assign(robot_state->get_vehicle().rotor_count(), 0.0);
  robot_state->set_rotor_speeds(rotor_speeds_);
  has_last_ = false;
  RCLCPP_INFO(get_logger(), "Controller [%s] loaded", plugin.c_str());
  return true;
}

bool
ControllersNode::cycle(std::shared_ptr<RobotState> robot_state)
{
  if (!controller_) {
    return false;
  }
  if (robot_state->consume_controller_reset()) {
    controller_->reset();
  }
  if (!is_flying(robot_state->get_mode())) {
    has_last_ = false;
    return true;
  }
  const auto seq = robot_state->get_estimate_seq();
  if (has_last_ && seq == last_seq_) {
    return true;  // No new estimate: keep the last command
  }
  const auto & estimate = robot_state->get_estimate();
  const double dt = std::clamp(
    has_last_ ? estimate.stamp - last_stamp_ : 0.0, min_dt_, max_dt_);
  last_seq_ = seq;
  last_stamp_ = estimate.stamp;
  has_last_ = true;

  if (!controller_->update(estimate, robot_state->get_setpoint(), dt, rotor_speeds_)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "The controller failed");
    return false;
  }
  robot_state->set_rotor_speeds(rotor_speeds_);
  return true;
}

void
ControllersNode::release()
{
  controller_.reset();
}

}  // namespace pilotaro
