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
#include <string>
#include <vector>

#include "pilotaro_controllers/CascadeController.hpp"
#include "pilotaro_controllers/UnderwaterController.hpp"

namespace pilotaro_controllers
{

// Standard BlueROV2 (6 thrusters, the vehicle shipped before the Heavy): 4 vectored
// horizontal thrusters below the center of mass and 2 vertical ones above it. Kept as the
// test case of a vehicle with one DOF (pitch) that the thrusters cannot produce
pilotaro::VehicleParams bluerov2()
{
  const double c = std::sqrt(0.5);
  pilotaro::VehicleParams v;
  v.mass = 10.012;
  v.inertia = {0.099, 0.178, 0.269};
  v.rotor_x = {0.14, 0.14, -0.14, -0.14, 0.0, 0.0};
  v.rotor_y = {-0.092, 0.092, -0.092, 0.092, -0.109, 0.109};
  v.rotor_z = {-0.011, -0.011, -0.011, -0.011, 0.066, 0.066};
  v.rotor_axis = {-c, -c, 0.0, -c, c, 0.0, c, -c, 0.0, c, c, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0};
  v.rotor_direction = {1.0, 1.0, -1.0, -1.0, 1.0, 1.0};
  v.motor_constant = 0.002;
  v.moment_constant = 0.0;
  v.min_rot_velocity = 0.0;
  v.max_rot_velocity = std::sqrt(40.0 / 0.002);
  v.reversible = true;
  return v;
}

// BlueROV2 Heavy (8 thrusters, pilotaro_description/config/bluerov2_heavy.params.yaml, same
// vehicle as the PX4 uuv_bluerov2_heavy model): 4 vectored horizontal thrusters 6 cm below
// the center of mass and 4 vertical ones at the corners, 4 and 7 pushing up, 5 and 6 down
pilotaro::VehicleParams bluerov2_heavy()
{
  const double c = std::sqrt(0.5);
  pilotaro::VehicleParams v;
  v.mass = 13.4;
  v.inertia = {0.263185, 0.22931, 0.375279};
  v.rotor_x = {0.14, 0.14, -0.14, -0.14, 0.12, 0.12, -0.12, -0.12};
  v.rotor_y = {-0.1, 0.1, -0.1, 0.1, -0.22, 0.22, -0.22, 0.22};
  v.rotor_z = {-0.059, -0.059, -0.059, -0.059, 0.001, 0.001, 0.001, 0.001};
  v.rotor_axis = {c, c, 0.0, c, -c, 0.0, c, -c, 0.0, c, c, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0};
  v.rotor_direction = {1.0, 1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0};
  v.motor_constant = 0.002;
  v.moment_constant = 0.0;
  v.min_rot_velocity = 0.0;
  v.max_rot_velocity = std::sqrt(49.69 / 0.002);
  v.reversible = true;
  return v;
}

class UnderwaterControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(controller_.configure(bluerov2(), gains_));
  }

  Wrench wrench()
  {
    EXPECT_TRUE(controller_.update(state_, setpoint_, 0.004, speeds_));
    return controller_.wrench_from_speeds(speeds_);
  }

  UnderwaterGains gains_;
  UnderwaterController controller_;
  pilotaro::VehicleState state_;
  pilotaro::ControlSetpoint setpoint_;
  std::vector<double> speeds_;
};

TEST_F(UnderwaterControllerTest, SixThrustersControlAllButPitch)
{
  const auto & c = controller_.controllable();
  EXPECT_TRUE(c[0]);   // surge
  EXPECT_TRUE(c[1]);   // sway
  EXPECT_TRUE(c[2]);   // heave
  EXPECT_TRUE(c[3]);   // roll (vertical thrusters on each side)
  EXPECT_FALSE(c[4]);  // pitch: left to the hydrostatic stability
  EXPECT_TRUE(c[5]);   // yaw
}

TEST_F(UnderwaterControllerTest, AtTheSetpointThrustersAreOff)
{
  const auto w = wrench();
  ASSERT_EQ(speeds_.size(), 6u);
  for (const double s : speeds_) {
    EXPECT_NEAR(s, 0.0, 1e-9);
  }
  EXPECT_NEAR(w.norm(), 0.0, 1e-9);
}

TEST_F(UnderwaterControllerTest, PositiveBuoyancyIsCompensatedDownwards)
{
  gains_.net_buoyancy = 2.0;
  ASSERT_TRUE(controller_.configure(bluerov2(), gains_));
  const auto w = wrench();
  EXPECT_NEAR(w[2], -2.0, 1e-6);
  EXPECT_NEAR(w.head<2>().norm(), 0.0, 1e-6);
}

TEST_F(UnderwaterControllerTest, ForwardErrorPushesForward)
{
  setpoint_.position.x() = 0.2;
  const auto w = wrench();
  // mass * (velocity P + one step of velocity I) on the 0.2 m/s velocity setpoint
  EXPECT_NEAR(w[0], 10.012 * (2.0 * 0.2 + 0.3 * 0.2 * 0.004), 1e-9);
  EXPECT_NEAR(w[1], 0.0, 1e-6);
  EXPECT_NEAR(w[2], 0.0, 1e-6);
  EXPECT_NEAR(w[5], 0.0, 1e-6);
}

TEST_F(UnderwaterControllerTest, DepthErrorPushesDown)
{
  setpoint_.position.z() = -0.3;
  const auto w = wrench();
  EXPECT_LT(w[2], -0.1);
  // vertical thrusters push along -z: positive speeds
  EXPECT_GT(speeds_[4], 0.0);
  EXPECT_GT(speeds_[5], 0.0);
}

TEST_F(UnderwaterControllerTest, HeadingErrorTurns)
{
  setpoint_.yaw = 0.5;
  const auto w = wrench();
  EXPECT_GT(w[5], 0.01);
  EXPECT_NEAR(w.head<3>().norm(), 0.0, 1e-6);
}

TEST_F(UnderwaterControllerTest, WorldSetpointIsRotatedToBody)
{
  // Facing north (yaw 90 deg), a goal to the north is forward in body frame
  state_.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  setpoint_.yaw = M_PI_2;
  setpoint_.position.y() = 0.2;
  const auto w = wrench();
  EXPECT_GT(w[0], 0.1);
  EXPECT_NEAR(w[1], 0.0, 1e-6);
}

TEST_F(UnderwaterControllerTest, SaturationKeepsTheWrenchDirection)
{
  Wrench requested;
  requested << 300.0, -100.0, 50.0, 0.0, 0.0, 5.0;
  controller_.allocate(requested, speeds_);
  double peak = 0.0;
  for (const double s : speeds_) {
    peak = std::max(peak, std::abs(s));
  }
  EXPECT_NEAR(peak, controller_.vehicle().max_rot_velocity, 1e-6);
  // Same direction in the controlled DOF (pitch is the result of the others)
  Wrench produced = controller_.wrench_from_speeds(speeds_);
  produced[4] = 0.0;
  EXPECT_GT(produced.normalized().dot(requested.normalized()), 0.999);
  EXPECT_LT(produced.norm(), requested.norm());
}

class HeavyControllerTest : public UnderwaterControllerTest
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(controller_.configure(bluerov2_heavy(), gains_));
  }
};

TEST_F(HeavyControllerTest, EightThrustersControlEveryDof)
{
  for (const bool c : controller_.controllable()) {
    EXPECT_TRUE(c);
  }
}

TEST_F(HeavyControllerTest, PitchErrorGivesPitchTorque)
{
  // Nose 0.2 rad down (positive pitch in FLU): the controller pitches it back up
  state_.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
  const auto w = wrench();
  EXPECT_LT(w[4], -0.1);
}

TEST_F(HeavyControllerTest, DepthErrorUsesTheFourVerticalThrusters)
{
  setpoint_.position.z() = -0.3;
  const auto w = wrench();
  EXPECT_LT(w[2], -0.1);
  // 4 and 7 push up (negative command to go down), 5 and 6 push down
  EXPECT_LT(speeds_[4], 0.0);
  EXPECT_GT(speeds_[5], 0.0);
  EXPECT_GT(speeds_[6], 0.0);
  EXPECT_LT(speeds_[7], 0.0);
  EXPECT_NEAR(w[3], 0.0, 1e-6);
  EXPECT_NEAR(w[4], 0.0, 1e-6);
}

TEST_F(HeavyControllerTest, SaturationKeepsTheFullWrenchDirection)
{
  Wrench requested;
  requested << 300.0, -100.0, 50.0, 1.0, -2.0, 5.0;
  controller_.allocate(requested, speeds_);
  const Wrench produced = controller_.wrench_from_speeds(speeds_);
  EXPECT_GT(produced.normalized().dot(requested.normalized()), 0.999);
  EXPECT_LT(produced.norm(), requested.norm());
}

TEST(UnderwaterController, RejectsThrustersWithoutHeave)
{
  auto vehicle = bluerov2();
  vehicle.rotor_axis[14] = 0.0;  // vertical thrusters made horizontal
  vehicle.rotor_axis[12] = 1.0;
  vehicle.rotor_axis[17] = 0.0;
  vehicle.rotor_axis[15] = 1.0;
  UnderwaterController controller;
  std::string error;
  EXPECT_FALSE(controller.configure(vehicle, UnderwaterGains(), &error));
  EXPECT_NE(error, "");
}

TEST(CascadeController, RejectsVectoredThrusters)
{
  CascadeController controller;
  std::string error;
  EXPECT_FALSE(controller.configure(bluerov2(), CascadeGains(), &error));
  EXPECT_NE(error.find("UnderwaterController"), std::string::npos);
}

}  // namespace pilotaro_controllers
