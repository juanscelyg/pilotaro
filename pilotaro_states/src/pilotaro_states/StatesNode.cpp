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

#include "pilotaro_states/StatesNode.hpp"

namespace pilotaro
{

using std::placeholders::_1;

StatesNode::StatesNode(
  const rclcpp::NodeOptions & options)
: Node("states_node", options)
{
  FlightModeParams p;
  p.takeoff_altitude = declare_parameter("takeoff_altitude", p.takeoff_altitude);
  p.takeoff_speed = declare_parameter("takeoff_speed", p.takeoff_speed);
  p.landing_speed = declare_parameter("landing_speed", p.landing_speed);
  p.land_detect_altitude = declare_parameter("land_detect_altitude", p.land_detect_altitude);
  p.land_detect_speed = declare_parameter("land_detect_speed", p.land_detect_speed);
  p.land_detect_time = declare_parameter("land_detect_time", p.land_detect_time);
  p.offboard_timeout = declare_parameter("offboard_timeout", p.offboard_timeout);
  p.estimate_timeout = declare_parameter("estimate_timeout", p.estimate_timeout);
  p.min_dt = declare_parameter("min_dt", p.min_dt);
  p.max_dt = declare_parameter("max_dt", p.max_dt);
  offboard_frame_id_ = declare_parameter("offboard_frame_id", std::string(""));
  machine_.configure(p);
  machine_.set_mode_callback(
    std::bind(&StatesNode::on_mode_change, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3));

  mode_pub_ = create_publisher<std_msgs::msg::String>(
    "mode", rclcpp::QoS(1).transient_local().reliable());
  pose_sp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "offboard/pose", 10, std::bind(&StatesNode::pose_setpoint_callback, this, _1));
  twist_sp_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "offboard/twist", 10, std::bind(&StatesNode::twist_setpoint_callback, this, _1));

  const auto service = [this](const std::string & name,
    std::function<CommandResult(RobotState &)> action) {
      return create_service<Trigger>(
        name, [this, name, action](
          [[maybe_unused]] const std::shared_ptr<Trigger::Request> request,
          std::shared_ptr<Trigger::Response> response) {command(name, response, action);});
    };
  arming_srv_ = service("arming", [this](RobotState & rs) {
        return machine_.arming(rs, now().seconds());
      });
  takeoff_srv_ = service("takeoff", [this](RobotState & rs) {return machine_.takeoff(rs);});
  landing_srv_ = service("landing", [this](RobotState & rs) {return machine_.landing(rs);});
  loiter_srv_ = service("loiter", [this](RobotState & rs) {return machine_.loiter(rs);});
  offboard_srv_ = service("offboard", [this](RobotState & rs) {
        return machine_.offboard(rs, now().seconds());
      });
}

StatesNode::~StatesNode()
{
}

bool
StatesNode::init(std::shared_ptr<RobotState> robot_state)
{
  robot_state_ = robot_state;
  machine_.reset(*robot_state_);
  return true;
}

bool
StatesNode::cycle(std::shared_ptr<RobotState> robot_state)
{
  machine_.update(*robot_state, now().seconds());
  return true;
}

void
StatesNode::release()
{
  robot_state_.reset();
}

void
StatesNode::command(
  const std::string & name, std::shared_ptr<Trigger::Response> response,
  const std::function<CommandResult(RobotState &)> & action)
{
  const CommandResult result = robot_state_ ?
    action(*robot_state_) : CommandResult{false, name + " rejected: pilotaro is not configured"};
  response->success = result.success;
  response->message = result.message;
  if (result.success) {
    RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", result.message.c_str());
  }
}

void
StatesNode::pose_setpoint_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!offboard_frame_id_.empty() && msg->header.frame_id != offboard_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring offboard/pose in frame '%s', expected '%s'",
      msg->header.frame_id.c_str(), offboard_frame_id_.c_str());
    return;
  }
  const auto & p = msg->pose.position;
  const auto & q = msg->pose.orientation;
  machine_.set_offboard_pose(
    {p.x, p.y, p.z}, yaw_from_quaternion(Eigen::Quaterniond(q.w, q.x, q.y, q.z)),
    now().seconds());
}

void
StatesNode::twist_setpoint_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  machine_.set_offboard_twist(
    {msg->linear.x, msg->linear.y, msg->linear.z}, msg->angular.z, now().seconds());
}

void
StatesNode::on_mode_change(FlightMode from, FlightMode to, const std::string & reason)
{
  if (from != to) {
    RCLCPP_INFO(
      get_logger(), "Mode %s -> %s (%s)", to_string(from).c_str(), to_string(to).c_str(),
      reason.c_str());
  }
  std_msgs::msg::String msg;
  msg.data = to_string(to);
  mode_pub_->publish(msg);
}

}  // namespace pilotaro
