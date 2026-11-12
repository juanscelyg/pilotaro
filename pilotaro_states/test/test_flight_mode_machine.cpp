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

#include <vector>

#include "pilotaro_states/FlightModeMachine.hpp"

namespace pilotaro
{

class FlightModeMachineTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    machine_.set_mode_callback(
      [this](FlightMode, FlightMode to, const std::string &) {modes_.push_back(to);});
    machine_.reset(state_);
  }

  // New estimate at time t with altitude z and vertical speed vz; then run the machine
  void step(double z, double vz = 0.0)
  {
    t_ += 0.01;
    VehicleState estimate;
    estimate.position.z() = z;
    estimate.velocity.z() = vz;
    estimate.stamp = t_;
    state_.set_estimate(estimate, t_);
    machine_.update(state_, t_);
  }

  FlightModeMachine machine_;
  RobotState state_;
  std::vector<FlightMode> modes_;
  double t_{0.0};
};

TEST_F(FlightModeMachineTest, StartsDisarmedAndNeedsEstimateToArm)
{
  EXPECT_EQ(state_.get_mode(), FlightMode::DISARMED);
  EXPECT_FALSE(machine_.arming(state_, t_).success);
  step(0.0);
  EXPECT_TRUE(machine_.arming(state_, t_).success);
  EXPECT_EQ(state_.get_mode(), FlightMode::ARMED);
  EXPECT_TRUE(machine_.arming(state_, t_).success);
  EXPECT_EQ(state_.get_mode(), FlightMode::DISARMED);
}

TEST_F(FlightModeMachineTest, TakeoffRampsToLoiter)
{
  step(0.0);
  ASSERT_TRUE(machine_.arming(state_, t_).success);
  ASSERT_TRUE(machine_.takeoff(state_).success);
  EXPECT_TRUE(state_.consume_controller_reset());
  EXPECT_EQ(state_.get_mode(), FlightMode::TAKEOFF);

  // The setpoint climbs at takeoff_speed while the vehicle follows it
  for (int i = 0; i < 400 && state_.get_mode() == FlightMode::TAKEOFF; ++i) {
    step(state_.get_setpoint().position.z());
  }
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  EXPECT_NEAR(state_.get_setpoint().position.z(), 2.0, 1e-9);
  EXPECT_FALSE(machine_.arming(state_, t_).success);
}

TEST_F(FlightModeMachineTest, NegativeTakeoffAltitudeDives)
{
  FlightModeParams params;
  params.takeoff_altitude = -2.0;
  params.takeoff_speed = 0.3;
  machine_.configure(params);
  step(0.0);
  ASSERT_TRUE(machine_.arming(state_, t_).success);
  ASSERT_TRUE(machine_.takeoff(state_).success);
  step(0.0);
  EXPECT_LT(state_.get_setpoint().position.z(), 0.0);
  EXPECT_DOUBLE_EQ(state_.get_setpoint().velocity.z(), -0.3);
  for (int i = 0; i < 1000 && state_.get_mode() == FlightMode::TAKEOFF; ++i) {
    step(state_.get_setpoint().position.z());
  }
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  EXPECT_NEAR(state_.get_setpoint().position.z(), -2.0, 1e-9);
}

TEST_F(FlightModeMachineTest, LandingDisarmsAfterTouchdown)
{
  step(0.0);
  machine_.arming(state_, t_);
  machine_.takeoff(state_);
  ASSERT_TRUE(machine_.landing(state_).success);
  EXPECT_FALSE(state_.get_setpoint().z_position);
  EXPECT_DOUBLE_EQ(state_.get_setpoint().velocity.z(), -0.5);
  for (int i = 0; i < 200 && state_.get_mode() == FlightMode::LANDING; ++i) {
    step(0.05);
  }
  EXPECT_EQ(state_.get_mode(), FlightMode::DISARMED);
}

TEST_F(FlightModeMachineTest, OffboardNeedsFreshSetpointAndTimesOut)
{
  step(0.0);
  machine_.arming(state_, t_);
  machine_.takeoff(state_);
  EXPECT_FALSE(machine_.offboard(state_, t_).success);

  machine_.set_offboard_pose({1.0, 2.0, 3.0}, 0.5, t_);
  ASSERT_TRUE(machine_.offboard(state_, t_).success);
  step(1.0);
  EXPECT_EQ(state_.get_mode(), FlightMode::OFFBOARD);
  EXPECT_DOUBLE_EQ(state_.get_setpoint().position.y(), 2.0);
  EXPECT_DOUBLE_EQ(state_.get_setpoint().yaw, 0.5);

  for (int i = 0; i < 60; ++i) {
    step(1.0);
  }
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
}

TEST_F(FlightModeMachineTest, TwistSetpointTracksVelocity)
{
  step(0.0);
  machine_.arming(state_, t_);
  machine_.set_offboard_twist({0.0, 0.0, 1.0}, 0.0, t_);
  ASSERT_TRUE(machine_.offboard(state_, t_).success);
  step(0.0);
  EXPECT_FALSE(state_.get_setpoint().z_position);
  EXPECT_DOUBLE_EQ(state_.get_setpoint().velocity.z(), 1.0);
}

TEST_F(FlightModeMachineTest, EstimateTimeoutDisarms)
{
  step(0.0);
  machine_.arming(state_, t_);
  machine_.takeoff(state_);
  machine_.update(state_, t_ + 1.0);
  EXPECT_EQ(state_.get_mode(), FlightMode::DISARMED);
}

TEST_F(FlightModeMachineTest, CommandsRejectedOnTheGround)
{
  step(0.0);
  EXPECT_FALSE(machine_.takeoff(state_).success);
  EXPECT_FALSE(machine_.landing(state_).success);
  EXPECT_FALSE(machine_.loiter(state_).success);
  machine_.set_offboard_pose({0.0, 0.0, 1.0}, 0.0, t_);
  EXPECT_FALSE(machine_.offboard(state_, t_).success);
}

}  // namespace pilotaro
