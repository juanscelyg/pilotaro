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

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "pilotaro_common/RobotState.hpp"
#include "pilotaro_estimators/EstimatorNode.hpp"
#include "pilotaro_estimators/OdometryEstimator.hpp"

namespace
{

nav_msgs::msg::Odometry yawed_odometry()
{
  // 90 deg yaw, moving forward (body x) at 1 m/s and yawing at 0.5 rad/s
  nav_msgs::msg::Odometry msg;
  msg.header.stamp.sec = 3;
  msg.header.frame_id = "odom";
  msg.pose.pose.position.z = 2.0;
  msg.pose.pose.orientation.z = std::sin(M_PI / 4.0);
  msg.pose.pose.orientation.w = std::cos(M_PI / 4.0);
  msg.twist.twist.linear.x = 1.0;
  msg.twist.twist.angular.z = 0.5;
  return msg;
}

}  // namespace

TEST(OdometryEstimator, BodyTwistIsRotatedToWorld)
{
  pilotaro::VehicleState state;
  pilotaro_estimators::OdometryEstimator::to_vehicle_state(yawed_odometry(), true, state);
  EXPECT_NEAR(state.velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(state.velocity.y(), 1.0, 1e-9);
  EXPECT_NEAR(state.angular_velocity.z(), 0.5, 1e-9);
  EXPECT_NEAR(state.position.z(), 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.stamp, 3.0);
}

TEST(OdometryEstimator, WorldTwistIsKept)
{
  pilotaro::VehicleState state;
  pilotaro_estimators::OdometryEstimator::to_vehicle_state(yawed_odometry(), false, state);
  EXPECT_NEAR(state.velocity.x(), 1.0, 1e-9);
  EXPECT_NEAR(state.velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(state.angular_velocity.z(), 0.5, 1e-9);
}

TEST(EstimatorNode, LoadsPluginAndPublishesEstimates)
{
  auto node = pilotaro::EstimatorNode::make_shared();
  auto robot_state = std::make_shared<pilotaro::RobotState>();
  ASSERT_TRUE(node->init(robot_state));

  auto publisher = node->create_publisher<nav_msgs::msg::Odometry>(
    "odometry", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_TRUE(node->cycle(robot_state));
  EXPECT_FALSE(robot_state->has_estimate());

  for (int i = 0; i < 50 && !robot_state->has_estimate(); ++i) {
    publisher->publish(yawed_odometry());
    executor.spin_some(std::chrono::milliseconds(10));
    node->cycle(robot_state);
  }
  ASSERT_TRUE(robot_state->has_estimate());
  EXPECT_NEAR(robot_state->get_estimate().position.z(), 2.0, 1e-9);

  // The same message is consumed only once
  const auto seq = robot_state->get_estimate_seq();
  node->cycle(robot_state);
  EXPECT_EQ(robot_state->get_estimate_seq(), seq);
}

TEST(EstimatorNode, UnknownPluginFails)
{
  auto node = pilotaro::EstimatorNode::make_shared(
    rclcpp::NodeOptions().parameter_overrides({{"estimator.plugin", "pilotaro_estimators/Nope"}}));
  EXPECT_FALSE(node->init(std::make_shared<pilotaro::RobotState>()));
  EXPECT_FALSE(node->cycle(std::make_shared<pilotaro::RobotState>()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
