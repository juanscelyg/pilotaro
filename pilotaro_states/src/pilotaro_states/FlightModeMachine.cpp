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
#include <cstdio>
#include <string>
#include <vector>

#include "pilotaro_states/FlightModeMachine.hpp"

namespace pilotaro
{

std::string to_string(FailsafeAction action)
{
  switch (action) {
    case FailsafeAction::NONE: return "none";
    case FailsafeAction::WARN: return "warn";
    case FailsafeAction::LOITER: return "loiter";
    case FailsafeAction::RTL: return "rtl";
    case FailsafeAction::LAND: return "land";
    case FailsafeAction::DISARM: return "disarm";
  }
  return "none";
}

bool parse_failsafe_action(const std::string & name, FailsafeAction & action)
{
  for (const auto a : {FailsafeAction::NONE, FailsafeAction::WARN, FailsafeAction::LOITER,
      FailsafeAction::RTL, FailsafeAction::LAND, FailsafeAction::DISARM})
  {
    if (to_string(a) == name) {
      action = a;
      return true;
    }
  }
  return false;
}

namespace
{
std::string join(const std::vector<std::string> & items, const char * sep)
{
  std::string out;
  for (const auto & i : items) {
    out += (out.empty() ? "" : sep) + i;
  }
  return out;
}

// Jerk limit of the acceleration feed-forward of mission legs [m/s^3]
constexpr double kMaxJerk = 3.0;

double tilt_of(const Eigen::Quaterniond & q)
{
  // Angle between the body z axis and the world z axis
  return std::acos(std::clamp(q.normalized().toRotationMatrix()(2, 2), -1.0, 1.0));
}
}  // namespace

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
  if (mode != FlightMode::MISSION) {
    mission_active_ = false;
  }
  if (mode != FlightMode::TAKEOFF) {
    mission_after_takeoff_ = false;
  }
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
  setpoint.xy_velocity = true;
  setpoint.z_position = true;
  setpoint.max_speed = 0.0;
  setpoint.acceleration.setZero();
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
  if (robot_state.get_mode() != FlightMode::DISARMED) {
    update_failsafes(robot_state, dt, now);
  }
}

void
FlightModeMachine::set_battery(double remaining, double now)
{
  has_battery_ = true;
  battery_remaining_ = remaining;
  battery_stamp_ = now;
}

void
FlightModeMachine::set_heartbeat(double now)
{
  has_heartbeat_ = true;
  heartbeat_stamp_ = now;
}

Eigen::Vector3d
FlightModeMachine::clamp_to_geofence(const Eigen::Vector3d & p) const
{
  const auto & f = params_.failsafe;
  Eigen::Vector3d out = p;
  if (f.geofence_radius > 0.0) {
    const Eigen::Vector2d d = p.head<2>() - home_.head<2>();
    if (d.norm() > f.geofence_radius) {
      out.head<2>() = home_.head<2>() + d * (f.geofence_radius / d.norm());
    }
  }
  if (f.geofence_max_height > 0.0) {
    out.z() = std::min(out.z(), home_.z() + f.geofence_max_height);
  }
  if (f.geofence_max_depth > 0.0) {
    out.z() = std::max(out.z(), home_.z() - f.geofence_max_depth);
  }
  return out;
}

std::string
FlightModeMachine::blocking_failsafe() const
{
  return active_action_ >= FailsafeAction::LOITER ? join(active_names_, ", ") : std::string();
}

void
FlightModeMachine::update_failsafes(RobotState & robot_state, double dt, double now)
{
  const auto & f = params_.failsafe;
  const auto & estimate = robot_state.get_estimate();
  std::vector<std::string> names;
  FailsafeAction action = FailsafeAction::NONE;
  const auto add = [&](const char * name, FailsafeAction a) {
      if (a != FailsafeAction::NONE) {
        names.emplace_back(name);
        action = std::max(action, a);
      }
    };

  if (has_battery_) {
    if (f.battery_timeout > 0.0 && now - battery_stamp_ > f.battery_timeout) {
      add("battery_lost", FailsafeAction::WARN);
    } else if (f.battery_critical > 0.0 && battery_remaining_ <= f.battery_critical) {
      add("battery_critical", f.battery_critical_action);
    } else if (f.battery_low > 0.0 && battery_remaining_ <= f.battery_low) {
      add("battery_low", f.battery_low_action);
    }
  }
  if ((clamp_to_geofence(estimate.position) - estimate.position).norm() >
    std::max(f.geofence_margin, 1e-6))
  {
    add("geofence", f.geofence_action);
  }
  const bool navigation_lost = f.max_position_std > 0.0 &&
    estimate.position_std > f.max_position_std;
  if (navigation_lost) {
    add("navigation", f.position_action);
  }
  if (f.heartbeat_timeout > 0.0 &&
    (!has_heartbeat_ || now - heartbeat_stamp_ > f.heartbeat_timeout))
  {
    add("heartbeat", f.heartbeat_action);
  }
  const bool flying = is_flying(robot_state.get_mode());
  if (flying && f.max_tilt > 0.0 && tilt_of(estimate.orientation) > f.max_tilt) {
    tilt_time_ += dt;
  } else {
    tilt_time_ = 0.0;
  }
  if (tilt_time_ > f.max_tilt_time) {
    add("attitude", f.attitude_action);
  }

  if (names != active_names_) {
    active_names_ = names;
    active_action_ = action;
    if (failsafe_callback_) {
      failsafe_callback_(active_names_, active_action_);
    }
  }
  if (!flying) {
    return;
  }
  const std::string reason = "failsafe: " + join(active_names_, ", ");
  const FlightMode mode = robot_state.get_mode();
  if (navigation_lost && action == FailsafeAction::RTL) {
    action = FailsafeAction::LAND;       // returning needs a valid position
  }
  switch (action) {
    case FailsafeAction::DISARM:
      set_mode(robot_state, FlightMode::DISARMED, reason);
      break;
    case FailsafeAction::LAND:
      if (mode != FlightMode::LANDING) {
        start_landing(robot_state, reason);
      }
      if (navigation_lost) {
        // The horizontal estimate is not valid: descend level, without horizontal control
        auto & setpoint = robot_state.get_setpoint();
        setpoint.xy_position = false;
        setpoint.xy_velocity = false;
      }
      break;
    case FailsafeAction::RTL:
      if (mode != FlightMode::RTL && mode != FlightMode::LANDING) {
        start_rtl(robot_state, reason);
      }
      break;
    case FailsafeAction::LOITER:
      if (mode == FlightMode::TAKEOFF || mode == FlightMode::OFFBOARD ||
        mode == FlightMode::MISSION)
      {
        hold_current_position(robot_state);
        set_mode(robot_state, FlightMode::LOITER, reason);
      }
      if (robot_state.get_mode() == FlightMode::LOITER) {
        // Hold inside the geofence
        auto & setpoint = robot_state.get_setpoint();
        setpoint.position = clamp_to_geofence(setpoint.position);
      }
      break;
    default:
      break;
  }
}

std::vector<std::string>
FlightModeMachine::prearm_failures(const RobotState & robot_state, double now) const
{
  const auto & f = params_.failsafe;
  std::vector<std::string> out;
  if (!robot_state.has_estimate() ||
    now - robot_state.get_estimate_reception() > params_.estimate_timeout)
  {
    out.emplace_back("no state estimate");
    return out;
  }
  const auto & estimate = robot_state.get_estimate();
  const auto fmt = [](double v, int decimals) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
      return std::string(buf);
    };
  const double tilt = tilt_of(estimate.orientation);
  if (f.max_tilt_arming > 0.0 && tilt > f.max_tilt_arming) {
    out.emplace_back("tilted " + fmt(tilt * 180.0 / M_PI, 0) + " deg");
  }
  if (f.max_rate_arming > 0.0 && estimate.angular_velocity.norm() > f.max_rate_arming) {
    out.emplace_back("rotating at " + fmt(estimate.angular_velocity.norm(), 2) + " rad/s");
  }
  if (f.max_position_std > 0.0 && estimate.position_std > f.max_position_std) {
    out.emplace_back("position uncertainty " + fmt(estimate.position_std, 1) + " m");
  }
  if (has_battery_) {
    if (f.battery_timeout > 0.0 && now - battery_stamp_ > f.battery_timeout) {
      out.emplace_back("no battery report");
    } else if (f.battery_min_arm > 0.0 && battery_remaining_ < f.battery_min_arm) {
      out.emplace_back("battery " + fmt(battery_remaining_ * 100.0, 0) + " % < " +
        fmt(f.battery_min_arm * 100.0, 0) + " %");
    }
  }
  if (f.heartbeat_timeout > 0.0 &&
    (!has_heartbeat_ || now - heartbeat_stamp_ > f.heartbeat_timeout))
  {
    out.emplace_back("no ground station heartbeat");
  }
  return out;
}

void
FlightModeMachine::update_setpoint(RobotState & robot_state, double dt, double now)
{
  const auto & estimate = robot_state.get_estimate();
  auto & setpoint = robot_state.get_setpoint();
  switch (robot_state.get_mode()) {
    case FlightMode::TAKEOFF: {
        // Move the altitude setpoint to the target, up (takeoff) or down (dive, negative
        // altitude): accelerate, cruise at takeoff_speed and brake, with the speed as
        // feedforward. The setpoint does not lead the estimate by more than
        // takeoff_max_lead, so nothing winds up while the vehicle is still on the ground.
        const double remaining = takeoff_target_z_ - setpoint.position.z();
        const double sign = remaining >= 0.0 ? 1.0 : -1.0;
        double speed = params_.takeoff_speed;
        if (params_.takeoff_acceleration > 0.0) {
          const double accelerating =
            std::abs(setpoint.velocity.z()) + params_.takeoff_acceleration * dt;
          // Fastest speed that can still stop at the target braking at takeoff_acceleration
          // in steps of dt (discrete form of sqrt(2 a d))
          const double a_dt = params_.takeoff_acceleration * dt;
          const double braking =
            a_dt * (std::sqrt(0.25 + 2.0 * std::abs(remaining) / (a_dt * dt)) - 0.5);
          speed = std::min({speed, accelerating, braking});
        }
        double step = speed * dt;
        if (params_.takeoff_max_lead > 0.0) {
          const double lead = sign * (setpoint.position.z() - estimate.position.z());
          step = std::clamp(params_.takeoff_max_lead - lead, 0.0, step);
          speed = std::min(speed, step / dt);
        }
        if (std::abs(remaining) > step && std::abs(remaining) > 1e-4) {
          setpoint.position.z() += sign * step;
          setpoint.velocity.z() = sign * speed;
        } else {
          setpoint.position.z() = takeoff_target_z_;
          setpoint.velocity.z() = 0.0;
          if (std::abs(estimate.position.z() - takeoff_target_z_) < 0.1) {
            if (mission_after_takeoff_) {
              start_waypoint(robot_state, 0);
              set_mode(robot_state, FlightMode::MISSION, "takeoff altitude reached");
              mission_active_ = true;
            } else {
              set_mode(robot_state, FlightMode::LOITER, "takeoff altitude reached");
            }
          }
        }
        break;
      }
    case FlightMode::LANDING:
      if (estimate.position.z() < home_.z() + params_.land_detect_altitude &&
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
      setpoint.xy_velocity = true;
      setpoint.max_speed = 0.0;
      setpoint.acceleration.setZero();
      if (offboard_type_ == OffboardType::POSE) {
        setpoint.xy_position = true;
        setpoint.z_position = true;
        setpoint.position = clamp_to_geofence(offboard_position_);
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
    case FlightMode::MISSION: {
        const auto & m = params_.mission;
        const Waypoint & wp = mission_[mission_index_];
        const bool at_goal = advance_leg(robot_state, dt, m.speed);
        if (!std::isnan(wp.yaw)) {
          setpoint.yaw = wp.yaw;
        }
        if (at_goal && (estimate.position - leg_goal_).norm() < m.acceptance_radius &&
          (m.acceptance_speed <= 0.0 || estimate.velocity.norm() < m.acceptance_speed))
        {
          hold_timer_ += dt;
          if (hold_timer_ >= m.hold_time) {
            if (mission_index_ + 1 < mission_.size()) {
              mission_index_ += 1;
              start_leg(robot_state, mission_[mission_index_].position, true);
            } else {
              finish(robot_state, m.end_action, "mission complete");
            }
          }
        }
        break;
      }
    case FlightMode::RTL: {
        const auto & m = params_.mission;
        const bool at_goal = advance_leg(robot_state, dt, m.rtl_speed);
        if (at_goal && (estimate.position - leg_goal_).norm() < m.acceptance_radius) {
          if (rtl_phase_ == RtlPhase::CLIMB) {
            rtl_phase_ = RtlPhase::RETURN;
            start_leg(robot_state, {home_.x(), home_.y(), leg_goal_.z()}, true);
          } else {
            finish(robot_state, m.rtl_final_action, "rtl: over the arming point");
            if (robot_state.get_mode() == FlightMode::LANDING) {
              // Touch down on the arming point, not where the acceptance radius was entered
              robot_state.get_setpoint().position.head<2>() = home_.head<2>();
            }
          }
        }
        break;
      }
    default:
      break;
  }
}

void
FlightModeMachine::start_leg(
  RobotState & robot_state, const Eigen::Vector3d & goal, bool chained)
{
  leg_point_ = chained ? leg_goal_ : robot_state.get_estimate().position;
  leg_goal_ = clamp_to_geofence(goal);
  leg_speed_ = 0.0;
  leg_acceleration_ = 0.0;
  hold_timer_ = 0.0;
  auto & setpoint = robot_state.get_setpoint();
  setpoint.xy_position = true;
  setpoint.xy_velocity = true;
  setpoint.z_position = true;
  setpoint.position = leg_point_;
  setpoint.velocity.setZero();
}

bool
FlightModeMachine::advance_leg(RobotState & robot_state, double dt, double speed)
{
  const auto & m = params_.mission;
  auto & setpoint = robot_state.get_setpoint();
  const Eigen::Vector3d to_goal = leg_goal_ - leg_point_;
  const double remaining = to_goal.norm();
  bool at_goal = remaining < 1e-6;
  if (!at_goal) {
    // Accelerate, cruise and brake (continuous sqrt(2 a d)); never lead the vehicle by
    // more than max_lead, so nothing winds up if it lags behind
    double v = speed;
    if (m.acceleration > 0.0) {
      v = std::min({v, leg_speed_ + m.acceleration * dt, std::sqrt(2.0 * m.acceleration * remaining)});
    }
    double step = v * dt;
    if (m.max_lead > 0.0 &&
      (leg_point_ - robot_state.get_estimate().position).norm() > m.max_lead)
    {
      step = 0.0;
    }
    const Eigen::Vector3d direction = to_goal / remaining;
    const double previous_speed = leg_speed_;
    if (step >= remaining) {
      leg_point_ = leg_goal_;
      leg_speed_ = 0.0;
      at_goal = true;
    } else {
      leg_point_ += step * direction;
      leg_speed_ = step / dt;
    }
    setpoint.velocity = leg_speed_ * direction;
    // Acceleration feed-forward of the profile, bounded (a stall for max_lead is not a
    // jump) and jerk limited: the trapezoidal profile steps its acceleration, and a step
    // would go straight to thrust and tilt
    double a = (leg_speed_ - previous_speed) / dt;
    if (m.acceleration > 0.0) {
      a = std::clamp(a, -m.acceleration, m.acceleration);
    }
    leg_acceleration_ += std::clamp(a - leg_acceleration_, -kMaxJerk * dt, kMaxJerk * dt);
    setpoint.acceleration = leg_acceleration_ * direction;
  } else {
    leg_acceleration_ += std::clamp(-leg_acceleration_, -kMaxJerk * dt, kMaxJerk * dt);
    setpoint.velocity.setZero();
    setpoint.acceleration.setZero();
  }
  setpoint.xy_position = true;
  setpoint.xy_velocity = true;
  setpoint.z_position = true;
  setpoint.max_speed = speed;
  setpoint.position = leg_point_;
  return at_goal;
}

void
FlightModeMachine::start_waypoint(RobotState & robot_state, std::size_t index)
{
  mission_index_ = index;
  start_leg(robot_state, mission_[index].position);
}

void
FlightModeMachine::finish(
  RobotState & robot_state, FailsafeAction action, const std::string & reason)
{
  switch (action) {
    case FailsafeAction::RTL:
      start_rtl(robot_state, reason, mission_active_);
      break;
    case FailsafeAction::LAND:
      start_landing(robot_state, reason);
      break;
    case FailsafeAction::DISARM:
      set_mode(robot_state, FlightMode::DISARMED, reason);
      break;
    default: {
        // Hold where the setpoint is (the last waypoint, or over home)
        auto & setpoint = robot_state.get_setpoint();
        setpoint.velocity.setZero();
        setpoint.acceleration.setZero();
        setpoint.max_speed = 0.0;
        set_mode(robot_state, FlightMode::LOITER, reason);
        break;
      }
  }
}

void
FlightModeMachine::start_rtl(
  RobotState & robot_state, const std::string & reason, bool chained)
{
  const auto & m = params_.mission;
  const Eigen::Vector3d p = chained ? leg_goal_ : robot_state.get_estimate().position;
  double z = p.z();
  if (m.rtl_altitude > 0.0) {
    z = std::max(z, home_.z() + m.rtl_altitude);
  }
  rtl_phase_ = RtlPhase::CLIMB;
  start_leg(robot_state, {p.x(), p.y(), z}, chained);
  set_mode(robot_state, FlightMode::RTL, reason);
}

CommandResult
FlightModeMachine::set_mission(const std::vector<Waypoint> & waypoints, FlightMode mode)
{
  if (mode == FlightMode::MISSION) {
    return {false, "Mission rejected: a mission is being flown"};
  }
  mission_ = waypoints;
  return {true, "Mission with " + std::to_string(waypoints.size()) + " waypoints"};
}

CommandResult
FlightModeMachine::mission(RobotState & robot_state)
{
  const FlightMode mode = robot_state.get_mode();
  if (mode == FlightMode::DISARMED) {
    return {false, "Mission rejected: vehicle is DISARMED"};
  }
  if (mission_.empty()) {
    return {false, "Mission rejected: publish the waypoints on mission first"};
  }
  if (const auto blocking = blocking_failsafe(); !blocking.empty()) {
    return {false, "Mission rejected: failsafe active (" + blocking + ")"};
  }
  for (std::size_t i = 0; i < mission_.size(); ++i) {
    if ((clamp_to_geofence(mission_[i].position) - mission_[i].position).norm() > 1e-6) {
      return {false, "Mission rejected: waypoint " + std::to_string(i) + " is outside the " +
        "geofence"};
    }
  }
  const std::string text = "Mission of " + std::to_string(mission_.size()) + " waypoints";
  if (mode == FlightMode::ARMED) {
    const auto result = takeoff(robot_state);
    if (!result.success) {
      return result;
    }
    mission_after_takeoff_ = true;
    return {true, text + " (taking off first)"};
  }
  start_waypoint(robot_state, 0);
  set_mode(robot_state, FlightMode::MISSION, "mission service");
  mission_active_ = true;
  return {true, text};
}

CommandResult
FlightModeMachine::rtl(RobotState & robot_state)
{
  if (!is_flying(robot_state.get_mode())) {
    return {false, "RTL rejected: vehicle is not flying (" +
      to_string(robot_state.get_mode()) + ")"};
  }
  start_rtl(robot_state, "rtl service");
  return {true, "Returning to the arming point"};
}

CommandResult
FlightModeMachine::arming(RobotState & robot_state, double now)
{
  switch (robot_state.get_mode()) {
    case FlightMode::DISARMED: {
        const auto failures = prearm_failures(robot_state, now);
        if (!failures.empty()) {
          return {false, "Arming rejected: " + join(failures, "; ")};
        }
        home_ = robot_state.get_estimate().position;
        tilt_time_ = 0.0;
        active_names_.clear();
        active_action_ = FailsafeAction::NONE;
        set_mode(robot_state, FlightMode::ARMED, "arming service");
        return {true, "Armed"};
      }
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
  if (const auto blocking = blocking_failsafe(); !blocking.empty()) {
    return {false, "Takeoff rejected: failsafe active (" + blocking + ")"};
  }
  robot_state.request_controller_reset();
  hold_current_position(robot_state);
  takeoff_target_z_ = clamp_to_geofence(
    robot_state.get_estimate().position +
    params_.takeoff_altitude * Eigen::Vector3d::UnitZ()).z();
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
  start_landing(robot_state, "landing service");
  return {true, "Landing"};
}

void
FlightModeMachine::start_landing(RobotState & robot_state, const std::string & reason)
{
  hold_current_position(robot_state);
  auto & setpoint = robot_state.get_setpoint();
  setpoint.z_position = false;
  setpoint.velocity.z() = -params_.landing_speed;
  set_mode(robot_state, FlightMode::LANDING, reason);
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
  if (const auto blocking = blocking_failsafe(); !blocking.empty()) {
    return {false, "Offboard rejected: failsafe active (" + blocking + ")"};
  }
  if (robot_state.get_mode() == FlightMode::ARMED) {
    robot_state.request_controller_reset();
  }
  hold_current_position(robot_state);
  set_mode(robot_state, FlightMode::OFFBOARD, "offboard service");
  return {true, "Offboard"};
}

}  // namespace pilotaro
