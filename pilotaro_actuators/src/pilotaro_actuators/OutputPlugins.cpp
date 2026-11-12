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

#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "pilotaro_actuators/OutputPlugins.hpp"

namespace pilotaro_actuators
{

bool
MotorSpeedOutput::on_initialize()
{
  const auto topic = declare_parameter<std::string>("topic", "command/motor_speed");
  pub_ = get_node()->create_publisher<actuator_msgs::msg::Actuators>(topic, 10);
  msg_.velocity.assign(get_vehicle().rotor_count(), 0.0);
  RCLCPP_INFO(get_logger(), "Rotor speeds on %s", pub_->get_topic_name());
  return true;
}

bool
MotorSpeedOutput::write(const std::vector<double> & rotor_speeds)
{
  msg_.header.stamp = get_node()->now();
  msg_.velocity = rotor_speeds;
  pub_->publish(msg_);
  return true;
}

bool
ThrusterOutput::on_initialize()
{
  const auto pattern = declare_parameter<std::string>("topic_pattern", "thruster_{}/cmd_thrust");
  const auto command = declare_parameter<std::string>("command", "thrust");
  if (command != "thrust" && command != "angular_velocity") {
    RCLCPP_ERROR(
      get_logger(), "command must be \"thrust\" or \"angular_velocity\", got \"%s\"",
      command.c_str());
    return false;
  }
  const auto placeholder = pattern.find("{}");
  if (placeholder == std::string::npos) {
    RCLCPP_ERROR(get_logger(), "topic_pattern must contain {} (the thruster index)");
    return false;
  }
  send_thrust_ = command == "thrust";
  pubs_.clear();
  for (std::size_t i = 0; i < get_vehicle().rotor_count(); ++i) {
    auto topic = pattern;
    topic.replace(placeholder, 2, std::to_string(i));
    pubs_.push_back(get_node()->create_publisher<std_msgs::msg::Float64>(topic, 10));
  }
  RCLCPP_INFO(
    get_logger(), "%s of %zu thrusters on %s ...", send_thrust_ ? "Thrust" : "Speed",
    pubs_.size(), pubs_.front()->get_topic_name());
  return true;
}

bool
ThrusterOutput::write(const std::vector<double> & rotor_speeds)
{
  if (rotor_speeds.size() != pubs_.size()) {
    return false;
  }
  const double k = get_vehicle().motor_constant;
  for (std::size_t i = 0; i < pubs_.size(); ++i) {
    const double w = rotor_speeds[i];
    msg_.data = send_thrust_ ? k * w * std::abs(w) : w;
    pubs_[i]->publish(msg_);
  }
  return true;
}

}  // namespace pilotaro_actuators

PLUGINLIB_EXPORT_CLASS(pilotaro_actuators::MotorSpeedOutput, pilotaro::ActuatorOutputBase)
PLUGINLIB_EXPORT_CLASS(pilotaro_actuators::ThrusterOutput, pilotaro::ActuatorOutputBase)
