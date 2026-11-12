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

#include <string>

#include "pilotaro_core/Parameters.hpp"
#include "pilotaro_system/PilotaroNode.hpp"

namespace pilotaro
{

PilotaroNode::PilotaroNode(const rclcpp::NodeOptions & options)
: LifecycleNode("pilotaro_node", options)
{
  RCLCPP_INFO(get_logger(), "Pilotaro Node Create");

  declare_parameter("robot_name", std::string(""));
  declare_parameter("simulation", false);
  declare_parameter("frequency", 250.0);
  declare_parameter("spin_time", 0.001);
  declare_parameter("real_time.enabled", true);
  declare_parameter("real_time.required", false);
  declare_parameter("real_time.priority", 80);
  declare_parameter("real_time.cpu", -1);
  declare_parameter("real_time.lock_memory", true);

  get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      std::bind(&PilotaroNode::preShutdown, this));
}

PilotaroNode::~PilotaroNode()
{
  RCLCPP_INFO(get_logger(), "Pilotaro state is %s", get_current_state().label().c_str());
  RCLCPP_INFO(get_logger(), "Pilotaro Node Destroyed");
}

void PilotaroNode::preShutdown()
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PilotaroNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Pilotaro on_configure");

  // The robot name identifies the platform in a multirobot system. By default it is the
  // namespace, which also separates the topics and services of each robot.
  robot_name_ = get_parameter("robot_name").as_string();
  if (robot_name_.empty()) {
    robot_name_ = get_namespace();
    robot_name_.erase(0, robot_name_.find_first_not_of('/'));
  }
  if (robot_name_.empty()) {
    robot_name_ = "pilotaro";
  }
  robot_state_->set_name(robot_name_);
  robot_state_->set_simulation(get_parameter("simulation").as_bool());

  try {
    const auto vehicle = declare_vehicle_params(get_node_parameters_interface());
    if (const auto error = vehicle.validate(); !error.empty()) {
      RCLCPP_ERROR(get_logger(), "Invalid vehicle parameters: %s", error.c_str());
      return CallbackReturnT::FAILURE;
    }
    robot_state_->set_vehicle(vehicle);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid vehicle parameters: %s", e.what());
    return CallbackReturnT::FAILURE;
  }

  if (!pilotaro_init()) {
    pilotaro_release();
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PilotaroNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Pilotaro on_activate");
  robot_state_->set_available(true);
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PilotaroNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Pilotaro on_deactive");
  robot_state_->set_available(false);
  actuators_node_->stop();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PilotaroNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Pilotaro state is %s", get_current_state().label().c_str());
  pilotaro_release();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PilotaroNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Pilotaro state is %s", get_current_state().label().c_str());
  actuators_node_->stop();
  pilotaro_release();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PilotaroNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  actuators_node_->stop();
  pilotaro_release();
  return CallbackReturnT::SUCCESS;
}

bool
PilotaroNode::pilotaro_init()
{
  if (!estimator_node_->init(robot_state_)) {
    RCLCPP_ERROR(get_logger(), "Unable to initialize [%s]", estimator_node_->get_name());
    return false;
  }
  if (!states_node_->init(robot_state_)) {
    RCLCPP_ERROR(get_logger(), "Unable to initialize [%s]", states_node_->get_name());
    return false;
  }
  if (!controllers_node_->init(robot_state_)) {
    RCLCPP_ERROR(get_logger(), "Unable to initialize [%s]", controllers_node_->get_name());
    return false;
  }
  if (!actuators_node_->init(robot_state_)) {
    RCLCPP_ERROR(get_logger(), "Unable to initialize [%s]", actuators_node_->get_name());
    return false;
  }

  robot_state_->show();
  return true;
}

void
PilotaroNode::pilotaro_release()
{
  estimator_node_->release();
  states_node_->release();
  controllers_node_->release();
  actuators_node_->release();
}

void
PilotaroNode::pilotaro_cycle()
{
  estimator_node_->cycle(robot_state_);
  states_node_->cycle(robot_state_);
  controllers_node_->cycle(robot_state_);
  actuators_node_->cycle(robot_state_);
}

std::map<std::string, PilotaroNodeInfo>
PilotaroNode::get_pilotaro_system_nodes()
{
  std::map<std::string, PilotaroNodeInfo> ret;

  ret[states_node_->get_name()] = {states_node_, nullptr};
  ret[estimator_node_->get_name()] = {estimator_node_, nullptr};
  ret[actuators_node_->get_name()] = {actuators_node_, nullptr};
  ret[controllers_node_->get_name()] = {controllers_node_, nullptr};

  return ret;
}

}  // namespace pilotaro
