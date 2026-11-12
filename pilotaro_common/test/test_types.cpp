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

#include "pilotaro_common/RobotState.hpp"

namespace pilotaro
{

TEST(VehicleParams, DefaultsAreValid)
{
  EXPECT_EQ(VehicleParams().validate(), "");
}

TEST(VehicleParams, RejectsMismatchedRotors)
{
  VehicleParams params;
  params.rotor_y.pop_back();
  EXPECT_NE(params.validate(), "");
}

TEST(VehicleParams, RejectsBadDirection)
{
  VehicleParams params;
  params.rotor_direction[0] = 0.5;
  EXPECT_NE(params.validate(), "");
}

TEST(VehicleParams, HoverBalancesWeight)
{
  const VehicleParams params;
  const double w = params.hover_rot_velocity();
  EXPECT_NEAR(4.0 * params.motor_constant * w * w, params.mass * params.gravity, 1e-9);
}

TEST(RobotState, EstimateSequence)
{
  RobotState state;
  EXPECT_FALSE(state.has_estimate());
  state.set_estimate(VehicleState(), 1.5);
  EXPECT_TRUE(state.has_estimate());
  EXPECT_EQ(state.get_estimate_seq(), 1u);
  EXPECT_DOUBLE_EQ(state.get_estimate_reception(), 1.5);
}

TEST(RobotState, ControllerResetIsConsumedOnce)
{
  RobotState state;
  state.request_controller_reset();
  EXPECT_TRUE(state.consume_controller_reset());
  EXPECT_FALSE(state.consume_controller_reset());
}

TEST(FlightMode, Flying)
{
  EXPECT_FALSE(is_flying(FlightMode::DISARMED));
  EXPECT_FALSE(is_flying(FlightMode::ARMED));
  EXPECT_TRUE(is_flying(FlightMode::TAKEOFF));
  EXPECT_TRUE(is_flying(FlightMode::OFFBOARD));
}

}  // namespace pilotaro
