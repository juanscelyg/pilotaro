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

#include "pilotaro_actuators/ActuatorsNode.hpp"

namespace pilotaro
{

ActuatorsNode::ActuatorsNode(
  const rclcpp::NodeOptions & options)
: Node("actuators_node", options),
  loader_("pilotaro_core", "pilotaro::ActuatorOutputBase")
{
  idle_rot_velocity_ = declare_parameter("idle_rot_velocity", 100.0);
  declare_parameter("output.plugin", std::string("pilotaro_actuators/MotorSpeedOutput"));
}

ActuatorsNode::~ActuatorsNode()
{
  output_.reset();
}

bool
ActuatorsNode::init(std::shared_ptr<RobotState> robot_state)
{
  const auto & vehicle = robot_state->get_vehicle();
  if (idle_rot_velocity_ < 0.0 || idle_rot_velocity_ > vehicle.max_rot_velocity) {
    RCLCPP_ERROR(
      get_logger(), "idle_rot_velocity (%.1f) must be in [0, %.1f]", idle_rot_velocity_,
      vehicle.max_rot_velocity);
    return false;
  }
  const auto plugin = get_parameter("output.plugin").as_string();
  try {
    output_ = loader_.createSharedInstance(plugin);
    if (!output_->initialize(shared_from_this(), "output", vehicle)) {
      RCLCPP_ERROR(get_logger(), "Output [%s] failed to initialize", plugin.c_str());
      output_.reset();
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Unable to load output [%s]: %s", plugin.c_str(), e.what());
    output_.reset();
    return false;
  }
  command_.assign(vehicle.rotor_count(), 0.0);
  RCLCPP_INFO(get_logger(), "Output [%s] loaded", plugin.c_str());
  return true;
}

bool
ActuatorsNode::cycle(std::shared_ptr<RobotState> robot_state)
{
  if (!output_) {
    return false;
  }
  const auto mode = robot_state->get_mode();
  const auto & speeds = robot_state->get_rotor_speeds();
  bool ok = true;
  if (mode == FlightMode::ARMED) {
    std::fill(command_.begin(), command_.end(), idle_rot_velocity_);
  } else if (is_flying(mode) && speeds.size() == command_.size()) {
    std::copy(speeds.begin(), speeds.end(), command_.begin());
  } else {
    // DISARMED, or a controller output of the wrong size
    ok = mode == FlightMode::DISARMED;
    std::fill(command_.begin(), command_.end(), 0.0);
  }
  if (!output_->write(command_)) {
    ok = false;
  }
  if (is_flying(mode) && speeds.size() != command_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Controller gave %zu rotor speeds, expected %zu",
      speeds.size(), command_.size());
  }
  return ok;
}

void
ActuatorsNode::stop()
{
  if (output_) {
    std::fill(command_.begin(), command_.end(), 0.0);
    output_->write(command_);
  }
}

void
ActuatorsNode::release()
{
  stop();
  output_.reset();
}

}  // namespace pilotaro
