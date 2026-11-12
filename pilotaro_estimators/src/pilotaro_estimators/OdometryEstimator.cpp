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

#include "pluginlib/class_list_macros.hpp"

#include "pilotaro_estimators/OdometryEstimator.hpp"

namespace pilotaro_estimators
{

using std::placeholders::_1;

bool
OdometryEstimator::on_initialize()
{
  const auto topic = declare_parameter<std::string>("topic", "odometry");
  twist_in_body_frame_ = declare_parameter("twist_in_body_frame", true);
  frame_id_ = declare_parameter<std::string>("frame_id", "");
  use_message_stamp_ = declare_parameter("use_message_stamp", true);

  odometry_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::SensorDataQoS(), std::bind(&OdometryEstimator::odometry_callback, this, _1));

  RCLCPP_INFO(
    get_logger(), "Odometry estimator on %s (twist in %s frame)",
    odometry_sub_->get_topic_name(), twist_in_body_frame_ ? "body" : "world");
  return true;
}

void
OdometryEstimator::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!frame_id_.empty() && msg->header.frame_id != frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_node()->get_clock(), 5000,
      "Ignoring odometry in frame '%s', expected '%s'", msg->header.frame_id.c_str(),
      frame_id_.c_str());
    return;
  }
  const double reception = get_node()->now().seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  last_msg_ = msg;
  reception_ = reception;
  fresh_ = true;
}

bool
OdometryEstimator::update(pilotaro::VehicleState & estimate)
{
  nav_msgs::msg::Odometry::ConstSharedPtr msg;
  double reception;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fresh_) {
      return false;
    }
    fresh_ = false;
    msg = last_msg_;
    reception = reception_;
  }
  to_vehicle_state(*msg, twist_in_body_frame_, estimate);
  if (!use_message_stamp_) {
    estimate.stamp = reception;
  }
  return true;
}

void
OdometryEstimator::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  fresh_ = false;
  last_msg_.reset();
}

void
OdometryEstimator::to_vehicle_state(
  const nav_msgs::msg::Odometry & msg, bool twist_in_body_frame, pilotaro::VehicleState & state)
{
  const auto & pose = msg.pose.pose;
  const auto & twist = msg.twist.twist;
  state.position = {pose.position.x, pose.position.y, pose.position.z};
  state.orientation = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).normalized();
  const Eigen::Vector3d linear(twist.linear.x, twist.linear.y, twist.linear.z);
  const Eigen::Vector3d angular(twist.angular.x, twist.angular.y, twist.angular.z);
  if (twist_in_body_frame) {
    state.velocity = state.orientation * linear;
    state.angular_velocity = angular;
  } else {
    state.velocity = linear;
    state.angular_velocity = state.orientation.conjugate() * angular;
  }
  state.stamp = rclcpp::Time(msg.header.stamp).seconds();
}

}  // namespace pilotaro_estimators

PLUGINLIB_EXPORT_CLASS(pilotaro_estimators::OdometryEstimator, pilotaro::StateEstimatorBase)
