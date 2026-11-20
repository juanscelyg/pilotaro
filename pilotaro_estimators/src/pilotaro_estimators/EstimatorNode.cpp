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

#include "pilotaro_estimators/EstimatorNode.hpp"

namespace pilotaro
{

EstimatorNode::EstimatorNode(
  const rclcpp::NodeOptions & options)
: Node("estimator_node", options),
  loader_("pilotaro_core", "pilotaro::StateEstimatorBase")
{
  declare_parameter("estimator.plugin", std::string("pilotaro_estimators/OdometryEstimator"));
}

EstimatorNode::~EstimatorNode()
{
  release();
}

bool
EstimatorNode::init(std::shared_ptr<RobotState> robot_state)
{
  const auto plugin = get_parameter("estimator.plugin").as_string();
  try {
    estimator_ = loader_.createSharedInstance(plugin);
    if (!estimator_->initialize(shared_from_this(), "estimator", robot_state->get_vehicle())) {
      RCLCPP_ERROR(get_logger(), "Estimator [%s] failed to initialize", plugin.c_str());
      estimator_.reset();
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Unable to load estimator [%s]: %s", plugin.c_str(), e.what());
    estimator_.reset();
    return false;
  }
  estimate_pub_ = create_publisher<nav_msgs::msg::Odometry>("estimate", rclcpp::SensorDataQoS());
  estimate_msg_.header.frame_id = "world";
  estimate_msg_.child_frame_id = "base_link";
  RCLCPP_INFO(get_logger(), "Estimator [%s] loaded", plugin.c_str());
  return true;
}

bool
EstimatorNode::cycle(std::shared_ptr<RobotState> robot_state)
{
  if (!estimator_) {
    return false;
  }
  if (estimator_->update(estimate_)) {
    robot_state->set_estimate(estimate_, now().seconds());
    if (estimate_pub_->get_subscription_count() > 0) {
      auto & m = estimate_msg_;
      m.header.stamp = rclcpp::Time(static_cast<int64_t>(estimate_.stamp * 1e9));
      m.pose.pose.position.x = estimate_.position.x();
      m.pose.pose.position.y = estimate_.position.y();
      m.pose.pose.position.z = estimate_.position.z();
      m.pose.pose.orientation.w = estimate_.orientation.w();
      m.pose.pose.orientation.x = estimate_.orientation.x();
      m.pose.pose.orientation.y = estimate_.orientation.y();
      m.pose.pose.orientation.z = estimate_.orientation.z();
      const Eigen::Vector3d v = estimate_.orientation.conjugate() * estimate_.velocity;
      m.twist.twist.linear.x = v.x();
      m.twist.twist.linear.y = v.y();
      m.twist.twist.linear.z = v.z();
      m.twist.twist.angular.x = estimate_.angular_velocity.x();
      m.twist.twist.angular.y = estimate_.angular_velocity.y();
      m.twist.twist.angular.z = estimate_.angular_velocity.z();
      // Horizontal uncertainty, split evenly between x and y (0 = not reported)
      m.pose.covariance[0] = m.pose.covariance[7] =
        0.5 * estimate_.position_std * estimate_.position_std;
      estimate_pub_->publish(m);
    }
  }
  return true;
}

void
EstimatorNode::release()
{
  estimator_.reset();
}

}  // namespace pilotaro
