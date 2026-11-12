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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "actuator_msgs/msg/actuators.hpp"
#include "std_msgs/msg/float64.hpp"

#include "pilotaro_actuators/ActuatorsNode.hpp"

namespace
{

// Spin until condition() or 1 s
template<typename Condition>
bool spin_until(rclcpp::Executor & executor, Condition condition)
{
  for (int i = 0; i < 100 && !condition(); ++i) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  return condition();
}

}  // namespace

TEST(ActuatorsNode, MotorSpeedOutputFollowsTheMode)
{
  auto node = pilotaro::ActuatorsNode::make_shared(
    rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/motors"}));
  auto robot_state = std::make_shared<pilotaro::RobotState>();
  ASSERT_TRUE(node->init(robot_state));

  std::vector<double> received;
  auto sub = node->create_subscription<actuator_msgs::msg::Actuators>(
    "command/motor_speed", 10,
    [&received](actuator_msgs::msg::Actuators::ConstSharedPtr msg) {received = msg->velocity;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // DISARMED: stopped, whatever the controller says
  robot_state->set_rotor_speeds({500.0, 500.0, 500.0, 500.0});
  ASSERT_TRUE(node->cycle(robot_state));
  ASSERT_TRUE(spin_until(executor, [&] {return received.size() == 4;}));
  EXPECT_DOUBLE_EQ(received[0], 0.0);

  // ARMED: idle
  robot_state->set_mode(pilotaro::FlightMode::ARMED);
  received.clear();
  ASSERT_TRUE(node->cycle(robot_state));
  ASSERT_TRUE(spin_until(executor, [&] {return !received.empty();}));
  EXPECT_DOUBLE_EQ(received[0], 100.0);

  // Flying: the controller output
  robot_state->set_mode(pilotaro::FlightMode::LOITER);
  received.clear();
  ASSERT_TRUE(node->cycle(robot_state));
  ASSERT_TRUE(spin_until(executor, [&] {return !received.empty();}));
  EXPECT_DOUBLE_EQ(received[3], 500.0);
}

TEST(ActuatorsNode, ThrusterOutputSendsSignedThrust)
{
  auto node = pilotaro::ActuatorsNode::make_shared(
    rclcpp::NodeOptions()
    .arguments({"--ros-args", "-r", "__ns:=/rov"})
    .parameter_overrides({
    {"output.plugin", "pilotaro_actuators/ThrusterOutput"},
    {"idle_rot_velocity", 0.0}}));
  auto robot_state = std::make_shared<pilotaro::RobotState>();
  pilotaro::VehicleParams vehicle;
  vehicle.motor_constant = 0.002;
  vehicle.min_rot_velocity = 0.0;
  vehicle.max_rot_velocity = 150.0;
  vehicle.reversible = true;
  robot_state->set_vehicle(vehicle);
  ASSERT_TRUE(node->init(robot_state));

  std::map<int, double> thrust;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subs;
  for (int i = 0; i < 4; ++i) {
    subs.push_back(
      node->create_subscription<std_msgs::msg::Float64>(
        "thruster_" + std::to_string(i) + "/cmd_thrust", 10,
        [&thrust, i](std_msgs::msg::Float64::ConstSharedPtr msg) {thrust[i] = msg->data;}));
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  robot_state->set_mode(pilotaro::FlightMode::OFFBOARD);
  robot_state->set_rotor_speeds({100.0, -100.0, 50.0, 0.0});
  ASSERT_TRUE(node->cycle(robot_state));
  ASSERT_TRUE(spin_until(executor, [&] {return thrust.size() == 4;}));
  EXPECT_NEAR(thrust[0], 20.0, 1e-9);   // 0.002 * 100^2
  EXPECT_NEAR(thrust[1], -20.0, 1e-9);  // reversed
  EXPECT_NEAR(thrust[2], 5.0, 1e-9);
  EXPECT_NEAR(thrust[3], 0.0, 1e-9);

  // stop(): every thruster to zero
  thrust.clear();
  node->stop();
  ASSERT_TRUE(spin_until(executor, [&] {return thrust.size() == 4;}));
  EXPECT_NEAR(thrust[1], 0.0, 1e-9);
}

TEST(ActuatorsNode, UnknownOutputFails)
{
  auto node = pilotaro::ActuatorsNode::make_shared(
    rclcpp::NodeOptions().parameter_overrides({{"output.plugin", "pilotaro_actuators/Nope"}}));
  EXPECT_FALSE(node->init(std::make_shared<pilotaro::RobotState>()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
