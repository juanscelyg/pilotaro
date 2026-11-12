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

#include <algorithm>
#include <cmath>
#include <string>

#include "pilotaro_states/FlightModeMachine.hpp"

namespace pilotaro
{

double yaw_from_quaternion(const Eigen::Quaterniond & q)
{
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void
FlightModeMachine::reset(RobotState & robot_state)
{
  offboard_type_ = OffboardType::NONE;
  has_last_ = false;
  set_mode(robot_state, FlightMode::DISARMED, "startup");
}

void
FlightModeMachine::set_mode(RobotState & robot_state, FlightMode mode, const std::string & reason)
{
  const FlightMode from = robot_state.get_mode();
  robot_state.set_mode(mode);
  landed_time_ = 0.0;
  if (mode_callback_) {
    mode_callback_(from, mode, reason);
  }
}

void
FlightModeMachine::hold_current_position(RobotState & robot_state)
{
  const auto & estimate = robot_state.get_estimate();
  auto & setpoint = robot_state.get_setpoint();
  setpoint.xy_position = true;
  setpoint.z_position = true;
  setpoint.position = estimate.position;
  setpoint.velocity.setZero();
  setpoint.yaw = yaw_from_quaternion(estimate.orientation);
}

bool
FlightModeMachine::offboard_setpoint_valid(double now) const
{
  return offboard_type_ != OffboardType::NONE && now - offboard_stamp_ < params_.offboard_timeout;
}

void
FlightModeMachine::set_offboard_pose(const Eigen::Vector3d & position, double yaw, double now)
{
  offboard_position_ = position;
  offboard_yaw_ = yaw;
  offboard_type_ = OffboardType::POSE;
  offboard_stamp_ = now;
}

void
FlightModeMachine::set_offboard_twist(
  const Eigen::Vector3d & velocity, double yaw_rate, double now)
{
  offboard_velocity_ = velocity;
  offboard_yaw_rate_ = yaw_rate;
  offboard_type_ = OffboardType::TWIST;
  offboard_stamp_ = now;
}

void
FlightModeMachine::update(RobotState & robot_state, double now)
{
  const FlightMode mode = robot_state.get_mode();
  if (mode == FlightMode::DISARMED) {
    return;
  }
  if (!robot_state.has_estimate() ||
    now - robot_state.get_estimate_reception() > params_.estimate_timeout)
  {
    set_mode(robot_state, FlightMode::DISARMED, "state estimate timeout");
    return;
  }
  const auto seq = robot_state.get_estimate_seq();
  if (has_last_ && seq == last_seq_) {
    return;
  }
  const double stamp = robot_state.get_estimate().stamp;
  const double dt = std::clamp(has_last_ ? stamp - last_stamp_ : 0.0, params_.min_dt,
      params_.max_dt);
  last_seq_ = seq;
  last_stamp_ = stamp;
  has_last_ = true;
  if (is_flying(mode)) {
    update_setpoint(robot_state, dt, now);
  }
}

void
FlightModeMachine::update_setpoint(RobotState & robot_state, double dt, double now)
{
  const auto & estimate = robot_state.get_estimate();
  auto & setpoint = robot_state.get_setpoint();
  switch (robot_state.get_mode()) {
    case FlightMode::TAKEOFF: {
        // Ramp the altitude to the target, up (takeoff) or down (dive, negative altitude)
        const double remaining = takeoff_target_z_ - setpoint.position.z();
        const double step = params_.takeoff_speed * dt;
        if (std::abs(remaining) > step) {
          setpoint.position.z() += std::copysign(step, remaining);
          setpoint.velocity.z() = std::copysign(params_.takeoff_speed, remaining);
        } else {
          setpoint.position.z() = takeoff_target_z_;
          setpoint.velocity.z() = 0.0;
          if (std::abs(estimate.position.z() - takeoff_target_z_) < 0.1) {
            set_mode(robot_state, FlightMode::LOITER, "takeoff altitude reached");
          }
        }
        break;
      }
    case FlightMode::LANDING:
      if (estimate.position.z() < home_z_ + params_.land_detect_altitude &&
        std::abs(estimate.velocity.z()) < params_.land_detect_speed)
      {
        landed_time_ += dt;
      } else {
        landed_time_ = 0.0;
      }
      if (landed_time_ > params_.land_detect_time) {
        set_mode(robot_state, FlightMode::DISARMED, "touchdown detected");
      }
      break;
    case FlightMode::OFFBOARD:
      if (!offboard_setpoint_valid(now)) {
        hold_current_position(robot_state);
        set_mode(robot_state, FlightMode::LOITER, "offboard setpoint timeout");
        break;
      }
      if (offboard_type_ == OffboardType::POSE) {
        setpoint.xy_position = true;
        setpoint.z_position = true;
        setpoint.position = offboard_position_;
        setpoint.velocity.setZero();
        setpoint.yaw = offboard_yaw_;
      } else {
        setpoint.xy_position = false;
        setpoint.z_position = false;
        setpoint.position = estimate.position;
        setpoint.velocity = offboard_velocity_;
        setpoint.yaw = std::remainder(setpoint.yaw + offboard_yaw_rate_ * dt, 2.0 * M_PI);
      }
      break;
    default:
      break;
  }
}

CommandResult
FlightModeMachine::arming(RobotState & robot_state, double now)
{
  switch (robot_state.get_mode()) {
    case FlightMode::DISARMED:
      if (!robot_state.has_estimate() ||
        now - robot_state.get_estimate_reception() > params_.estimate_timeout)
      {
        return {false, "Arming rejected: no state estimate"};
      }
      home_z_ = robot_state.get_estimate().position.z();
      set_mode(robot_state, FlightMode::ARMED, "arming service");
      return {true, "Armed"};
    case FlightMode::ARMED:
      set_mode(robot_state, FlightMode::DISARMED, "arming service");
      return {true, "Disarmed"};
    default:
      return {false, "Disarming rejected: vehicle is flying (" +
        to_string(robot_state.get_mode()) + ")"};
  }
}

CommandResult
FlightModeMachine::takeoff(RobotState & robot_state)
{
  if (robot_state.get_mode() != FlightMode::ARMED) {
    return {false, "Takeoff rejected: vehicle must be ARMED, is " +
      to_string(robot_state.get_mode())};
  }
  robot_state.request_controller_reset();
  hold_current_position(robot_state);
  takeoff_target_z_ = robot_state.get_estimate().position.z() + params_.takeoff_altitude;
  set_mode(robot_state, FlightMode::TAKEOFF, "takeoff service");
  return {true, "Taking off to z = " + std::to_string(takeoff_target_z_) + " m"};
}

CommandResult
FlightModeMachine::landing(RobotState & robot_state)
{
  if (!is_flying(robot_state.get_mode())) {
    return {false, "Landing rejected: vehicle is not flying (" +
      to_string(robot_state.get_mode()) + ")"};
  }
  hold_current_position(robot_state);
  auto & setpoint = robot_state.get_setpoint();
  setpoint.z_position = false;
  setpoint.velocity.z() = -params_.landing_speed;
  set_mode(robot_state, FlightMode::LANDING, "landing service");
  return {true, "Landing"};
}

CommandResult
FlightModeMachine::loiter(RobotState & robot_state)
{
  if (!is_flying(robot_state.get_mode())) {
    return {false, "Loiter rejected: vehicle is not flying (" +
      to_string(robot_state.get_mode()) + ")"};
  }
  hold_current_position(robot_state);
  set_mode(robot_state, FlightMode::LOITER, "loiter service");
  return {true, "Loitering"};
}

CommandResult
FlightModeMachine::offboard(RobotState & robot_state, double now)
{
  if (robot_state.get_mode() == FlightMode::DISARMED) {
    return {false, "Offboard rejected: vehicle is DISARMED"};
  }
  if (!offboard_setpoint_valid(now)) {
    return {false, "Offboard rejected: publish setpoints on offboard/pose or offboard/twist first"};
  }
  if (robot_state.get_mode() == FlightMode::ARMED) {
    robot_state.request_controller_reset();
  }
  hold_current_position(robot_state);
  set_mode(robot_state, FlightMode::OFFBOARD, "offboard service");
  return {true, "Offboard"};
}

}  // namespace pilotaro
