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
#include <vector>

#include "pilotaro_controllers/CascadeController.hpp"
#include "pilotaro_controllers/Shaping.hpp"

namespace pilotaro_controllers
{

TEST(LowPass3, FirstSamplePassesAndConstantIsKept)
{
  LowPass3 filter(5.0);
  const Eigen::Vector3d x(1.0, -2.0, 3.0);
  EXPECT_TRUE(filter.update(x, 0.004).isApprox(x));
  for (int i = 0; i < 100; ++i) {
    filter.update(x, 0.004);
  }
  EXPECT_TRUE(filter.update(x, 0.004).isApprox(x));
}

TEST(LowPass3, AttenuatesSampleToSampleNoise)
{
  // +-1 alternating at 125 Hz (250 Hz sampling) through a 5 Hz filter
  LowPass3 filter(5.0);
  filter.update(Eigen::Vector3d::Zero(), 0.004);
  double peak = 0.0;
  for (int i = 0; i < 500; ++i) {
    const double s = (i % 2 == 0) ? 1.0 : -1.0;
    peak = std::max(peak, std::abs(filter.update(Eigen::Vector3d::Constant(s), 0.004).x()));
  }
  EXPECT_LT(peak, 0.15);
}

TEST(LowPass3, ZeroCutoffDoesNotFilter)
{
  LowPass3 filter(0.0);
  filter.update(Eigen::Vector3d::Zero(), 0.004);
  EXPECT_DOUBLE_EQ(filter.update(Eigen::Vector3d::Ones(), 0.004).x(), 1.0);
}

TEST(PositionToVelocity, PointsStraightAtTheGoal)
{
  pilotaro::ControlSetpoint sp;
  sp.position = Eigen::Vector3d(2.0, 1.0, 1.0);
  const Eigen::Vector3d gain(1.0, 1.0, 1.5);   // different per axis
  const Eigen::Vector3d v = position_to_velocity(sp, Eigen::Vector3d::Zero(), gain, 10.0, 10.0);
  EXPECT_NEAR(v.normalized().dot(sp.position.normalized()), 1.0, 1e-12);
}

TEST(PositionToVelocity, SingleAxisKeepsItsGain)
{
  pilotaro::ControlSetpoint sp;
  sp.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  const Eigen::Vector3d v =
    position_to_velocity(sp, Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 1.0, 1.5), 3.0, 1.5);
  EXPECT_NEAR(v.z(), 0.6, 1e-12);
  EXPECT_NEAR(v.head<2>().norm(), 0.0, 1e-12);
}

TEST(PositionToVelocity, NearTheGoalEachAxisKeepsItsGain)
{
  pilotaro::ControlSetpoint sp;
  sp.position = Eigen::Vector3d(0.02, 0.0, 0.02);       // 3 cm away
  const Eigen::Vector3d gain(1.0, 1.0, 1.5);
  const Eigen::Vector3d v = position_to_velocity(sp, Eigen::Vector3d::Zero(), gain, 10.0, 10.0);
  EXPECT_NEAR(v.x(), 0.02 * (0.9717 * 1.0 + 0.0283 * 1.25), 1e-4);
  EXPECT_NEAR(v.z(), 0.02 * (0.9717 * 1.5 + 0.0283 * 1.25), 1e-4);
}

TEST(PositionToVelocity, LimitsScaleTheWholeVector)
{
  pilotaro::ControlSetpoint sp;
  sp.position = Eigen::Vector3d(10.0, 0.0, 10.0);
  const Eigen::Vector3d v =
    position_to_velocity(sp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), 3.0, 1.5);
  EXPECT_NEAR(v.z(), 1.5, 1e-12);       // the vertical limit is the binding one
  EXPECT_NEAR(v.x(), 1.5, 1e-12);       // same scale: still at 45 degrees
}

TEST(PositionToVelocity, VelocityControlOnAnAxisKeepsPerAxisLimits)
{
  pilotaro::ControlSetpoint sp;
  sp.xy_position = false;   // velocity control in the plane
  sp.velocity = Eigen::Vector3d(5.0, 0.0, 0.0);
  sp.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  const Eigen::Vector3d v =
    position_to_velocity(sp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), 3.0, 1.5);
  EXPECT_NEAR(v.x(), 3.0, 1e-12);
  EXPECT_NEAR(v.z(), 1.0, 1e-12);
}

TEST(PositionToVelocity, MaxSpeedCapsBothModes)
{
  pilotaro::ControlSetpoint sp;
  sp.position = Eigen::Vector3d(10.0, 0.0, 10.0);
  sp.max_speed = 1.0;
  Eigen::Vector3d v =
    position_to_velocity(sp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), 3.0, 1.5);
  EXPECT_NEAR(v.norm(), 1.0, 1e-12);
  EXPECT_NEAR(v.x(), v.z(), 1e-12);
  sp.xy_position = false;
  sp.velocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  v = position_to_velocity(sp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), 3.0, 1.5);
  EXPECT_NEAR(v.norm(), 1.0, 1e-12);
}

TEST(CascadeController, FilteredRateNoiseBarelyMovesTheMotors)
{
  pilotaro::VehicleParams v;
  v.mass = 2.0;
  v.inertia = {0.02, 0.02, 0.04};
  v.rotor_x = {0.17, -0.17, 0.17, -0.17};
  v.rotor_y = {-0.17, 0.17, 0.17, -0.17};
  v.rotor_direction = {1.0, 1.0, -1.0, -1.0};
  v.motor_constant = 8.54858e-06;
  v.moment_constant = 0.016;
  v.max_rot_velocity = 1000.0;
  v.min_rot_velocity = 100.0;
  // Mean change of the rotor speeds with +-0.05 rad/s gyro noise, without and with filter
  const auto activity = [&](double cutoff) {
      CascadeGains g;
      g.angular_rate_cutoff = cutoff;
      CascadeController c;
      EXPECT_TRUE(c.configure(v, g));
      pilotaro::VehicleState state;
      pilotaro::ControlSetpoint sp;
      std::vector<double> speeds, last;
      double change = 0.0;
      for (int i = 0; i < 500; ++i) {
        state.angular_velocity = Eigen::Vector3d::Constant((i % 2 == 0) ? 0.05 : -0.05);
        c.update(state, sp, 0.004, speeds);
        if (!last.empty()) {
          change += std::abs(speeds[0] - last[0]);
        }
        last = speeds;
      }
      return change;
    };
  EXPECT_LT(activity(30.0), 0.5 * activity(0.0));
}

}  // namespace pilotaro_controllers
