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
#include <string>
#include <vector>

#include "pilotaro_controllers/CascadeController.hpp"
#include "pilotaro_controllers/ControllersNode.hpp"

namespace pilotaro_controllers
{

// Symmetric quadrotor X: rotors at the same distance from the center of mass
pilotaro::VehicleParams symmetric_quad()
{
  pilotaro::VehicleParams vehicle;
  vehicle.rotor_x = {0.174, -0.174, 0.174, -0.174};
  vehicle.rotor_y = {-0.174, 0.174, 0.174, -0.174};
  return vehicle;
}

class CascadeControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(controller_.configure(symmetric_quad(), CascadeGains()));
  }

  double hover_thrust() const
  {
    return controller_.vehicle().mass * controller_.vehicle().gravity;
  }

  CascadeController controller_;
  pilotaro::VehicleState state_;
  pilotaro::ControlSetpoint setpoint_;
  std::vector<double> speeds_;
};

TEST_F(CascadeControllerTest, HoverGivesEqualRotorSpeeds)
{
  ASSERT_TRUE(controller_.update(state_, setpoint_, 0.004, speeds_));
  ASSERT_EQ(speeds_.size(), 4u);
  for (const double s : speeds_) {
    EXPECT_NEAR(s, controller_.vehicle().hover_rot_velocity(), 1e-6);
  }
}

TEST_F(CascadeControllerTest, PositiveYawTorqueSpeedsUpClockwiseRotors)
{
  controller_.allocate({hover_thrust(), 0.0, 0.0, 0.5}, speeds_);
  // rotors 0, 1 are ccw; 2, 3 are cw
  EXPECT_LT(speeds_[0], speeds_[2]);
  EXPECT_LT(speeds_[1], speeds_[3]);
}

TEST_F(CascadeControllerTest, PositiveRollTorqueSpeedsUpLeftRotors)
{
  controller_.allocate({hover_thrust(), 0.5, 0.0, 0.0}, speeds_);
  // rotors 1, 2 have y > 0
  EXPECT_GT(speeds_[1], speeds_[0]);
  EXPECT_GT(speeds_[2], speeds_[3]);
}

TEST_F(CascadeControllerTest, ForwardSetpointPitchesNoseDown)
{
  setpoint_.position.x() = 1.0;
  ASSERT_TRUE(controller_.update(state_, setpoint_, 0.004, speeds_));
  // rear rotors (1, 3) must push harder than front rotors (0, 2)
  EXPECT_GT(speeds_[1] + speeds_[3], speeds_[0] + speeds_[2]);
}

TEST_F(CascadeControllerTest, ClimbSetpointIncreasesThrust)
{
  setpoint_.position.z() = 1.0;
  ASSERT_TRUE(controller_.update(state_, setpoint_, 0.004, speeds_));
  for (const double s : speeds_) {
    EXPECT_GT(s, controller_.vehicle().hover_rot_velocity());
  }
}

TEST_F(CascadeControllerTest, SpeedsAreSaturated)
{
  setpoint_.z_position = false;
  setpoint_.velocity.z() = 100.0;
  state_.velocity.z() = -100.0;
  ASSERT_TRUE(controller_.update(state_, setpoint_, 0.004, speeds_));
  for (const double s : speeds_) {
    EXPECT_LE(s, controller_.vehicle().max_rot_velocity);
  }
}

TEST_F(CascadeControllerTest, RejectsInvalidTimeStep)
{
  EXPECT_FALSE(controller_.update(state_, setpoint_, 0.0, speeds_));
}

TEST(CascadeController, ForwardCenterOfMassLoadsFrontRotors)
{
  // Default vehicle: the cameras move the center of mass forward
  const pilotaro::VehicleParams vehicle;
  CascadeController controller;
  ASSERT_TRUE(controller.configure(vehicle, CascadeGains()));
  std::vector<double> speeds;
  ASSERT_TRUE(controller.update({}, {}, 0.004, speeds));
  // front rotors (0, 2) closer to the center of mass carry more weight
  EXPECT_GT(speeds[0], speeds[1]);
  EXPECT_GT(speeds[2], speeds[3]);
  double thrust = 0.0;
  for (const double s : speeds) {
    thrust += vehicle.motor_constant * s * s;
  }
  EXPECT_NEAR(thrust, vehicle.mass * vehicle.gravity, 1e-6);
}

// Parrot Bebop 2 (pilotaro_description/config/bebop2.params.yaml): rotors in Parrot order,
// front right ccw, front left cw, rear left ccw, rear right cw
pilotaro::VehicleParams bebop2()
{
  pilotaro::VehicleParams vehicle;
  vehicle.mass = 0.503;
  vehicle.inertia = {0.00196259, 0.00187869, 0.00359966};
  vehicle.rotor_x = {0.0846291, 0.0846291, -0.0902809, -0.0902809};
  vehicle.rotor_y = {-0.114353, 0.114627, 0.114517, -0.114233};
  vehicle.rotor_direction = {1.0, -1.0, 1.0, -1.0};
  vehicle.motor_constant = 1.9006e-06;
  vehicle.moment_constant = 0.010891;
  vehicle.max_rot_velocity = 1250.0;
  return vehicle;
}

TEST(CascadeController, Bebop2HoversWithMargin)
{
  const auto vehicle = bebop2();
  CascadeController controller;
  ASSERT_TRUE(controller.configure(vehicle, CascadeGains()));
  std::vector<double> speeds;
  ASSERT_TRUE(controller.update({}, {}, 0.004, speeds));
  double thrust = 0.0;
  for (const double s : speeds) {
    thrust += vehicle.motor_constant * s * s;
    // hover around 800 rad/s, far from saturation
    EXPECT_GT(s, 700.0);
    EXPECT_LT(s, 0.75 * vehicle.max_rot_velocity);
  }
  EXPECT_NEAR(thrust, vehicle.mass * vehicle.gravity, 1e-6);
  // The front rotors are closer to the center of mass: they carry more weight
  EXPECT_GT(speeds[0], speeds[3]);
  EXPECT_GT(speeds[1], speeds[2]);
}

TEST(CascadeController, Bebop2TorquesFollowParrotOrder)
{
  const auto vehicle = bebop2();
  CascadeController controller;
  ASSERT_TRUE(controller.configure(vehicle, CascadeGains()));
  const double weight = vehicle.mass * vehicle.gravity;
  std::vector<double> hover, speeds;
  controller.allocate({weight, 0.0, 0.0, 0.0}, hover);

  // Positive yaw torque: the cw rotors (1 front left, 3 rear right) speed up
  controller.allocate({weight, 0.0, 0.0, 0.01}, speeds);
  EXPECT_GT(speeds[1], hover[1]);
  EXPECT_GT(speeds[3], hover[3]);
  EXPECT_LT(speeds[0], hover[0]);
  EXPECT_LT(speeds[2], hover[2]);

  // Positive roll torque (left side up): left rotors (1, 2) speed up
  controller.allocate({weight, 0.01, 0.0, 0.0}, speeds);
  EXPECT_GT(speeds[1], hover[1]);
  EXPECT_GT(speeds[2], hover[2]);
  EXPECT_LT(speeds[0], hover[0]);
  EXPECT_LT(speeds[3], hover[3]);

  // Positive pitch torque (nose down): rear rotors (2, 3) speed up
  controller.allocate({weight, 0.0, 0.01, 0.0}, speeds);
  EXPECT_GT(speeds[2], hover[2]);
  EXPECT_GT(speeds[3], hover[3]);
  EXPECT_LT(speeds[0], hover[0]);
  EXPECT_LT(speeds[1], hover[1]);
}

TEST(CascadeController, HexarotorHovers)
{
  pilotaro::VehicleParams hexa;
  hexa.rotor_x.clear();
  hexa.rotor_y.clear();
  hexa.rotor_direction.clear();
  for (int i = 0; i < 6; ++i) {
    const double a = i * M_PI / 3.0;
    hexa.rotor_x.push_back(0.3 * std::cos(a));
    hexa.rotor_y.push_back(0.3 * std::sin(a));
    hexa.rotor_direction.push_back(i % 2 == 0 ? 1.0 : -1.0);
  }
  CascadeController controller;
  ASSERT_TRUE(controller.configure(hexa, CascadeGains()));
  std::vector<double> speeds;
  ASSERT_TRUE(controller.update({}, {}, 0.004, speeds));
  ASSERT_EQ(speeds.size(), 6u);
  for (const double s : speeds) {
    EXPECT_NEAR(s, hexa.hover_rot_velocity(), 1e-6);
  }
}

TEST(CascadeController, RejectsCollinearRotors)
{
  pilotaro::VehicleParams line;
  line.rotor_y = {0.0, 0.0, 0.0, 0.0};
  CascadeController controller;
  std::string error;
  EXPECT_FALSE(controller.configure(line, CascadeGains(), &error));
  EXPECT_NE(error, "");
}

}  // namespace pilotaro_controllers

TEST(ControllersNode, RunsOnlyWhileFlyingAndOnNewEstimates)
{
  auto node = pilotaro::ControllersNode::make_shared();
  auto robot_state = std::make_shared<pilotaro::RobotState>();
  ASSERT_TRUE(node->init(robot_state));

  pilotaro::VehicleState estimate;
  estimate.stamp = 1.0;
  robot_state->set_estimate(estimate, 1.0);

  // ARMED: the controller does not run
  robot_state->set_mode(pilotaro::FlightMode::ARMED);
  ASSERT_TRUE(node->cycle(robot_state));
  EXPECT_DOUBLE_EQ(robot_state->get_rotor_speeds()[0], 0.0);

  // Flying with a setpoint above: thrust over hover
  robot_state->set_mode(pilotaro::FlightMode::LOITER);
  robot_state->get_setpoint().position.z() = 1.0;
  ASSERT_TRUE(node->cycle(robot_state));
  const double first = robot_state->get_rotor_speeds()[0];
  EXPECT_GT(first, robot_state->get_vehicle().hover_rot_velocity());

  // Same estimate: the command is kept
  robot_state->get_setpoint().position.z() = 0.0;
  ASSERT_TRUE(node->cycle(robot_state));
  EXPECT_DOUBLE_EQ(robot_state->get_rotor_speeds()[0], first);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
