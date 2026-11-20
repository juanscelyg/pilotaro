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

#include <algorithm>
#include <cmath>
#include <string>
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
    VehicleState estimate;
    estimate.position.z() = z;
    estimate.velocity.z() = vz;
    step(estimate);
  }

  // New full estimate; then run the machine
  void step(VehicleState estimate)
  {
    t_ += 0.01;
    estimate.stamp = t_;
    state_.set_estimate(estimate, t_);
    machine_.update(state_, t_);
  }

  // Arm, take off and loiter at 2 m
  void fly()
  {
    step(0.0);
    ASSERT_TRUE(machine_.arming(state_, t_).success);
    ASSERT_TRUE(machine_.takeoff(state_).success);
    for (int i = 0; i < 800 && state_.get_mode() == FlightMode::TAKEOFF; ++i) {
      step(state_.get_setpoint().position.z());
    }
    ASSERT_EQ(state_.get_mode(), FlightMode::LOITER);
  }

  // The vehicle follows the setpoint exactly for at most n steps, while in mode
  void follow(FlightMode mode, int n)
  {
    for (int i = 0; i < n && state_.get_mode() == mode; ++i) {
      step(at(state_.get_setpoint().position));
    }
  }

  // At position p, still and level
  VehicleState at(const Eigen::Vector3d & p)
  {
    VehicleState e;
    e.position = p;
    return e;
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

  // The setpoint climbs (up to takeoff_speed) while the vehicle follows it
  for (int i = 0; i < 800 && state_.get_mode() == FlightMode::TAKEOFF; ++i) {
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
  EXPECT_LT(state_.get_setpoint().velocity.z(), 0.0);
  double fastest = 0.0;
  for (int i = 0; i < 1000 && state_.get_mode() == FlightMode::TAKEOFF; ++i) {
    step(state_.get_setpoint().position.z());
    fastest = std::min(fastest, state_.get_setpoint().velocity.z());
  }
  EXPECT_NEAR(fastest, -0.3, 1e-9);  // cruises at takeoff_speed
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  EXPECT_NEAR(state_.get_setpoint().position.z(), -2.0, 1e-9);
}

TEST_F(FlightModeMachineTest, TakeoffAcceleratesAndBrakes)
{
  step(0.0);
  ASSERT_TRUE(machine_.arming(state_, t_).success);
  ASSERT_TRUE(machine_.takeoff(state_).success);
  double previous = 0.0;
  double max_change = 0.0;
  for (int i = 0; i < 800 && state_.get_mode() == FlightMode::TAKEOFF; ++i) {
    step(state_.get_setpoint().position.z());
    const double v = state_.get_setpoint().velocity.z();
    max_change = std::max(max_change, std::abs(v - previous));
    previous = v;
  }
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  // speed changes by takeoff_acceleration * dt per step (two at most on the last step,
  // which lands on the target): no jump from takeoff_speed to 0 at the end
  const double a = FlightModeParams().takeoff_acceleration;
  EXPECT_LE(max_change, 2.0 * a * 0.01 + 1e-9);
  EXPECT_NEAR(state_.get_setpoint().velocity.z(), 0.0, 1e-9);
}

TEST_F(FlightModeMachineTest, TakeoffSetpointWaitsForTheVehicle)
{
  step(0.0);
  ASSERT_TRUE(machine_.arming(state_, t_).success);
  ASSERT_TRUE(machine_.takeoff(state_).success);
  // The vehicle stays on the ground: the setpoint stops takeoff_max_lead above it
  for (int i = 0; i < 300; ++i) {
    step(0.0);
  }
  EXPECT_NEAR(state_.get_setpoint().position.z(), 0.3, 1e-9);
  EXPECT_NEAR(state_.get_setpoint().velocity.z(), 0.0, 1e-9);
  EXPECT_EQ(state_.get_mode(), FlightMode::TAKEOFF);
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

TEST_F(FlightModeMachineTest, PrearmChecksExplainEveryFailure)
{
  FlightModeParams params;
  params.failsafe.heartbeat_timeout = 1.0;
  machine_.configure(params);
  VehicleState e;
  e.orientation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX());   // 29 deg
  e.angular_velocity.z() = 1.0;
  e.position_std = 3.0;
  machine_.set_battery(0.2, t_);
  step(e);
  const auto result = machine_.arming(state_, t_);
  EXPECT_FALSE(result.success);
  for (const char * reason : {"tilted 29 deg", "rotating", "position uncertainty 3.0 m",
      "battery 20 % < 30 %", "no ground station heartbeat"})
  {
    EXPECT_NE(result.message.find(reason), std::string::npos) << result.message;
  }
  machine_.set_battery(0.9, t_);
  machine_.set_heartbeat(t_);
  step(0.0);
  EXPECT_TRUE(machine_.arming(state_, t_).success) << machine_.arming(state_, t_).message;
}

TEST_F(FlightModeMachineTest, LowBatteryWarnsAndCriticalLands)
{
  fly();
  machine_.set_battery(0.15, t_);
  step(2.0);
  ASSERT_EQ(machine_.active_failsafes(), std::vector<std::string>{"battery_low"});
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);      // only a warning
  machine_.set_battery(0.05, t_);
  step(2.0);
  EXPECT_EQ(state_.get_mode(), FlightMode::LANDING);
  EXPECT_EQ(machine_.failsafe_action(), FailsafeAction::LAND);
  // Commands that would undo the failsafe are rejected
  machine_.set_offboard_pose({1.0, 0.0, 2.0}, 0.0, t_);
  const auto result = machine_.offboard(state_, t_);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("battery_critical"), std::string::npos);
  EXPECT_EQ(state_.get_mode(), FlightMode::LANDING);
}

TEST_F(FlightModeMachineTest, BatteryReportsMustKeepArriving)
{
  fly();
  machine_.set_battery(0.9, t_);
  for (int i = 0; i < 300; ++i) {
    step(2.0);
  }
  EXPECT_EQ(machine_.active_failsafes(), std::vector<std::string>{"battery_lost"});
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
}

TEST_F(FlightModeMachineTest, GeofenceClampsSetpointsAndHoldsInside)
{
  FlightModeParams params;
  params.failsafe.geofence_radius = 3.0;
  params.failsafe.geofence_max_height = 5.0;
  machine_.configure(params);
  fly();
  // Offboard goals beyond the fence stop at it
  machine_.set_offboard_pose({10.0, 0.0, 9.0}, 0.0, t_);
  ASSERT_TRUE(machine_.offboard(state_, t_).success);
  step(at({0.0, 0.0, 2.0}));
  EXPECT_NEAR(state_.get_setpoint().position.x(), 3.0, 1e-9);
  EXPECT_NEAR(state_.get_setpoint().position.z(), 5.0, 1e-9);
  // Pushed out (or flying a velocity setpoint out): hold inside the fence
  step(at({3.5, 0.0, 2.0}));
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  EXPECT_LE(state_.get_setpoint().position.head<2>().norm(), 3.0 + 1e-9);
  EXPECT_FALSE(machine_.offboard(state_, t_).success);
  // Back inside, the failsafe clears and offboard is accepted again
  step(at({2.5, 0.0, 2.0}));
  EXPECT_TRUE(machine_.active_failsafes().empty());
  machine_.set_offboard_pose({1.0, 0.0, 2.0}, 0.0, t_);
  EXPECT_TRUE(machine_.offboard(state_, t_).success);
}

TEST_F(FlightModeMachineTest, DepthLimitForUnderwaterVehicles)
{
  FlightModeParams params;
  params.takeoff_altitude = -5.0;
  params.failsafe.geofence_max_depth = 3.0;
  machine_.configure(params);
  step(0.0);
  ASSERT_TRUE(machine_.arming(state_, t_).success);
  ASSERT_TRUE(machine_.takeoff(state_).success);
  for (int i = 0; i < 2000 && state_.get_mode() == FlightMode::TAKEOFF; ++i) {
    step(state_.get_setpoint().position.z());
  }
  // The dive stops at the depth limit
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  EXPECT_NEAR(state_.get_setpoint().position.z(), -3.0, 1e-9);
}

TEST_F(FlightModeMachineTest, LostNavigationLands)
{
  fly();
  VehicleState e = at({0.0, 0.0, 2.0});
  e.position_std = 0.5;
  step(e);
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  e.position_std = 2.0;
  step(e);
  EXPECT_EQ(state_.get_mode(), FlightMode::LANDING);
  EXPECT_EQ(modes_.back(), FlightMode::LANDING);
  // Without a valid horizontal estimate it descends level, without horizontal control
  EXPECT_FALSE(state_.get_setpoint().xy_position);
  EXPECT_FALSE(state_.get_setpoint().xy_velocity);
  EXPECT_FALSE(state_.get_setpoint().z_position);
}

TEST_F(FlightModeMachineTest, LostGroundStationLoiters)
{
  fly();
  FlightModeParams params;
  params.failsafe.heartbeat_timeout = 1.0;
  machine_.configure(params);       // enabled in flight (a parameter change)
  machine_.set_heartbeat(t_);
  machine_.set_offboard_pose({1.0, 0.0, 2.0}, 0.0, t_);
  ASSERT_TRUE(machine_.offboard(state_, t_).success);
  for (int i = 0; i < 50; ++i) {
    machine_.set_heartbeat(t_);
    machine_.set_offboard_pose({1.0, 0.0, 2.0}, 0.0, t_);
    step(2.0);
  }
  EXPECT_EQ(state_.get_mode(), FlightMode::OFFBOARD);
  for (int i = 0; i < 120; ++i) {
    machine_.set_offboard_pose({1.0, 0.0, 2.0}, 0.0, t_);
    step(2.0);
  }
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  EXPECT_EQ(machine_.active_failsafes(), std::vector<std::string>{"heartbeat"});
}

TEST_F(FlightModeMachineTest, SustainedFlipDisarmsButAShortOneDoesNot)
{
  fly();
  VehicleState e = at({0.0, 0.0, 2.0});
  e.orientation = Eigen::AngleAxisd(1.4, Eigen::Vector3d::UnitY());
  for (int i = 0; i < 20; ++i) {       // 0.2 s
    step(e);
  }
  step(2.0);
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  for (int i = 0; i < 40 && state_.get_mode() != FlightMode::DISARMED; ++i) {
    step(e);
  }
  EXPECT_EQ(state_.get_mode(), FlightMode::DISARMED);
}

TEST_F(FlightModeMachineTest, FailsafeChangesAreReported)
{
  std::vector<std::vector<std::string>> reports;
  machine_.set_failsafe_callback(
    [&](const std::vector<std::string> & active, FailsafeAction) {reports.push_back(active);});
  fly();
  machine_.set_battery(0.15, t_);
  step(2.0);
  step(2.0);       // no change: no report
  machine_.set_battery(0.9, t_);
  step(2.0);
  ASSERT_EQ(reports.size(), 2u);
  EXPECT_EQ(reports[0], std::vector<std::string>{"battery_low"});
  EXPECT_TRUE(reports[1].empty());
}

std::vector<Waypoint> square()
{
  std::vector<Waypoint> w(3);
  w[0].position = {4.0, 0.0, 3.0};
  w[1].position = {4.0, 4.0, 3.0};
  w[1].yaw = 1.0;
  w[2].position = {0.0, 4.0, 2.0};
  return w;
}

// Distance from p to the segment a-b
double to_segment(const Eigen::Vector3d & p, const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  const double t = std::clamp((p - a).dot(b - a) / (b - a).squaredNorm(), 0.0, 1.0);
  return (a + t * (b - a) - p).norm();
}

TEST_F(FlightModeMachineTest, MissionFromTheGroundTakesOffAndFliesEveryLeg)
{
  EXPECT_TRUE(machine_.set_mission(square(), state_.get_mode()).success);
  step(0.0);
  ASSERT_TRUE(machine_.arming(state_, t_).success);
  const auto result = machine_.mission(state_);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(state_.get_mode(), FlightMode::TAKEOFF);
  follow(FlightMode::TAKEOFF, 2000);
  ASSERT_EQ(state_.get_mode(), FlightMode::MISSION);

  std::vector<int> visited;
  double max_speed = 0.0, max_off_leg = 0.0;
  Eigen::Vector3d from = state_.get_estimate().position;
  for (int i = 0; i < 5000 && state_.get_mode() == FlightMode::MISSION; ++i) {
    const int index = machine_.mission_index();
    if (visited.empty() || visited.back() != index) {
      if (!visited.empty()) {
        from = square()[visited.back()].position;
      }
      visited.push_back(index);
    }
    max_speed = std::max(max_speed, state_.get_setpoint().velocity.norm());
    max_off_leg = std::max(
      max_off_leg, to_segment(state_.get_setpoint().position, from, square()[index].position));
    step(at(state_.get_setpoint().position));
  }
  EXPECT_EQ(visited, (std::vector<int>{0, 1, 2}));
  EXPECT_LE(max_speed, 2.0 + 1e-9);        // mission.speed
  EXPECT_LT(max_off_leg, 1e-3);            // straight legs
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);      // end_action
  EXPECT_TRUE((state_.get_setpoint().position - square()[2].position).norm() < 1e-9);
  EXPECT_NEAR(state_.get_setpoint().yaw, 1.0, 1e-9);     // kept after waypoint 1
  EXPECT_EQ(machine_.mission_index(), -1);
}

TEST_F(FlightModeMachineTest, RtlClimbsReturnsAndLands)
{
  FlightModeParams params;
  params.mission.end_action = FailsafeAction::RTL;
  machine_.configure(params);
  machine_.set_mission(square(), state_.get_mode());
  fly();
  ASSERT_TRUE(machine_.mission(state_).success);
  follow(FlightMode::MISSION, 5000);
  ASSERT_EQ(state_.get_mode(), FlightMode::RTL);
  double max_z = 0.0;
  for (int i = 0; i < 5000 && state_.get_mode() == FlightMode::RTL; ++i) {
    max_z = std::max(max_z, state_.get_setpoint().position.z());
    step(at(state_.get_setpoint().position));
  }
  EXPECT_NEAR(max_z, 5.0, 1e-9);       // home (z 0) + rtl_altitude
  EXPECT_EQ(state_.get_mode(), FlightMode::LANDING);
  EXPECT_LT(state_.get_estimate().position.head<2>().norm(), 0.3);
  // It lands on the arming point itself
  EXPECT_LT(state_.get_setpoint().position.head<2>().norm(), 1e-9);
}

TEST_F(FlightModeMachineTest, RtlNeverDescendsToTheReturnHeight)
{
  FlightModeParams params;
  params.mission.rtl_final_action = FailsafeAction::LOITER;
  machine_.configure(params);
  fly();
  machine_.set_offboard_pose({3.0, 0.0, 8.0}, 0.0, t_);
  ASSERT_TRUE(machine_.offboard(state_, t_).success);
  step(at({3.0, 0.0, 8.0}));
  ASSERT_TRUE(machine_.rtl(state_).success);
  double min_z = 100.0;
  for (int i = 0; i < 5000 && state_.get_mode() == FlightMode::RTL; ++i) {
    min_z = std::min(min_z, state_.get_setpoint().position.z());
    step(at(state_.get_setpoint().position));
  }
  EXPECT_NEAR(min_z, 8.0, 1e-9);
  EXPECT_EQ(state_.get_mode(), FlightMode::LOITER);
  EXPECT_LT(state_.get_setpoint().position.head<2>().norm(), 1e-9);
}

TEST_F(FlightModeMachineTest, MissionOutsideTheGeofenceIsRejected)
{
  FlightModeParams params;
  params.failsafe.geofence_radius = 7.0;
  machine_.configure(params);
  auto w = square();
  w.push_back(Waypoint{{9.0, 0.0, 2.0}, std::nan("")});
  machine_.set_mission(w, state_.get_mode());
  fly();
  const auto result = machine_.mission(state_);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("waypoint 3"), std::string::npos) << result.message;
}

TEST_F(FlightModeMachineTest, MissionCannotBeReplacedWhileFlyingIt)
{
  machine_.set_mission(square(), state_.get_mode());
  fly();
  ASSERT_TRUE(machine_.mission(state_).success);
  EXPECT_FALSE(machine_.set_mission(square(), state_.get_mode()).success);
}

TEST_F(FlightModeMachineTest, SetpointWaitsForAVehicleThatLags)
{
  machine_.set_mission(square(), state_.get_mode());
  fly();
  ASSERT_TRUE(machine_.mission(state_).success);
  for (int i = 0; i < 500; ++i) {
    step(at({0.0, 0.0, 2.0}));        // the vehicle does not move
  }
  EXPECT_LE((state_.get_setpoint().position - Eigen::Vector3d(0.0, 0.0, 2.0)).norm(), 1.0 + 0.03);
}

TEST_F(FlightModeMachineTest, FailsafeCanReturnHomeButNotWithoutNavigation)
{
  FlightModeParams params;
  params.failsafe.battery_critical_action = FailsafeAction::RTL;
  params.failsafe.position_action = FailsafeAction::RTL;
  machine_.configure(params);
  fly();
  machine_.set_battery(0.05, t_);
  step(2.0);
  EXPECT_EQ(state_.get_mode(), FlightMode::RTL);
  EXPECT_FALSE(machine_.mission(state_).success);
  // Without a valid position a return is not possible: it lands instead
  VehicleState e = at(state_.get_setpoint().position);
  e.position_std = 5.0;
  step(e);
  EXPECT_EQ(state_.get_mode(), FlightMode::LANDING);
}

TEST(FailsafeAction, NamesRoundTrip)
{
  for (const auto a : {FailsafeAction::NONE, FailsafeAction::WARN, FailsafeAction::LOITER,
      FailsafeAction::RTL, FailsafeAction::LAND, FailsafeAction::DISARM})
  {
    FailsafeAction parsed;
    ASSERT_TRUE(parse_failsafe_action(to_string(a), parsed));
    EXPECT_EQ(parsed, a);
  }
  FailsafeAction parsed;
  EXPECT_FALSE(parse_failsafe_action("return", parsed));
  EXPECT_EQ(to_string(FlightMode::MISSION), "MISSION");
  EXPECT_EQ(to_string(FlightMode::RTL), "RTL");
  EXPECT_TRUE(is_flying(FlightMode::MISSION));
  EXPECT_TRUE(is_flying(FlightMode::RTL));
}

}  // namespace pilotaro
