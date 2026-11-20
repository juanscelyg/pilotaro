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
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pilotaro_states/StatesNode.hpp"

namespace pilotaro
{

using std::placeholders::_1;

StatesNode::StatesNode(
  const rclcpp::NodeOptions & options)
: Node("states_node", options)
{
  FlightModeParams p;
  p.takeoff_altitude = declare_parameter("takeoff_altitude", p.takeoff_altitude);
  p.takeoff_speed = declare_parameter("takeoff_speed", p.takeoff_speed);
  p.takeoff_acceleration = declare_parameter("takeoff_acceleration", p.takeoff_acceleration);
  p.takeoff_max_lead = declare_parameter("takeoff_max_lead", p.takeoff_max_lead);
  p.landing_speed = declare_parameter("landing_speed", p.landing_speed);
  p.land_detect_altitude = declare_parameter("land_detect_altitude", p.land_detect_altitude);
  p.land_detect_speed = declare_parameter("land_detect_speed", p.land_detect_speed);
  p.land_detect_time = declare_parameter("land_detect_time", p.land_detect_time);
  p.offboard_timeout = declare_parameter("offboard_timeout", p.offboard_timeout);
  p.estimate_timeout = declare_parameter("estimate_timeout", p.estimate_timeout);
  p.min_dt = declare_parameter("min_dt", p.min_dt);
  p.max_dt = declare_parameter("max_dt", p.max_dt);
  offboard_frame_id_ = declare_parameter("offboard_frame_id", std::string(""));
  {
    // Declare the failsafe.* parameters with their defaults, then read them
    const FailsafeParams d;
    declare_parameter("failsafe.battery_min_arm", d.battery_min_arm);
    declare_parameter("failsafe.battery_low", d.battery_low);
    declare_parameter("failsafe.battery_low_action", to_string(d.battery_low_action));
    declare_parameter("failsafe.battery_critical", d.battery_critical);
    declare_parameter("failsafe.battery_critical_action", to_string(d.battery_critical_action));
    declare_parameter("failsafe.battery_timeout", d.battery_timeout);
    declare_parameter("failsafe.geofence_radius", d.geofence_radius);
    declare_parameter("failsafe.geofence_max_height", d.geofence_max_height);
    declare_parameter("failsafe.geofence_max_depth", d.geofence_max_depth);
    declare_parameter("failsafe.geofence_margin", d.geofence_margin);
    declare_parameter("failsafe.geofence_action", to_string(d.geofence_action));
    declare_parameter("failsafe.max_position_std", d.max_position_std);
    declare_parameter("failsafe.position_action", to_string(d.position_action));
    declare_parameter("failsafe.heartbeat_timeout", d.heartbeat_timeout);
    declare_parameter("failsafe.heartbeat_action", to_string(d.heartbeat_action));
    declare_parameter("failsafe.max_tilt", d.max_tilt);
    declare_parameter("failsafe.max_tilt_time", d.max_tilt_time);
    declare_parameter("failsafe.attitude_action", to_string(d.attitude_action));
    declare_parameter("failsafe.max_tilt_arming", d.max_tilt_arming);
    declare_parameter("failsafe.max_rate_arming", d.max_rate_arming);
    const MissionParams m;
    declare_parameter("mission.speed", m.speed);
    declare_parameter("mission.acceleration", m.acceleration);
    declare_parameter("mission.acceptance_radius", m.acceptance_radius);
    declare_parameter("mission.acceptance_speed", m.acceptance_speed);
    declare_parameter("mission.hold_time", m.hold_time);
    declare_parameter("mission.max_lead", m.max_lead);
    declare_parameter("mission.end_action", to_string(m.end_action));
    declare_parameter("mission.rtl_altitude", m.rtl_altitude);
    declare_parameter("mission.rtl_speed", m.rtl_speed);
    declare_parameter("mission.rtl_final_action", to_string(m.rtl_final_action));
    std::string error;
    if (!read_failsafe_params(p.failsafe, error) || !read_mission_params(p.mission, error)) {
      throw std::invalid_argument(error);
    }
  }
  machine_.configure(p);
  machine_.set_failsafe_callback(
    std::bind(&StatesNode::on_failsafe_change, this, std::placeholders::_1,
    std::placeholders::_2));
  // The failsafe parameters can be changed at run time (ros2 param set)
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&StatesNode::on_parameters, this, std::placeholders::_1));
  post_param_cb_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      for (const auto & prm : parameters) {
        if (prm.get_name().rfind("failsafe.", 0) == 0 ||
        prm.get_name().rfind("mission.", 0) == 0)
        {
          FlightModeParams p = machine_.params();
          std::string error;
          if (read_failsafe_params(p.failsafe, error) && read_mission_params(p.mission, error)) {
            machine_.configure(p);
            RCLCPP_INFO(get_logger(), "Failsafe and mission parameters updated");
          }
          return;
        }
      }
    });
  machine_.set_mode_callback(
    std::bind(&StatesNode::on_mode_change, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3));

  mode_pub_ = create_publisher<std_msgs::msg::String>(
    "mode", rclcpp::QoS(1).transient_local().reliable());
  failsafe_pub_ = create_publisher<std_msgs::msg::String>(
    "failsafe", rclcpp::QoS(1).transient_local().reliable());
  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
    "battery", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::BatteryState::ConstSharedPtr m) {
      if (std::isfinite(m->percentage)) {
        machine_.set_battery(m->percentage, now().seconds());
      }
    });
  heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
    "heartbeat", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) {
      machine_.set_heartbeat(now().seconds());
    });
  mission_current_pub_ = create_publisher<std_msgs::msg::Int32>(
    "mission/current", rclcpp::QoS(1).transient_local().reliable());
  mission_sub_ = create_subscription<nav_msgs::msg::Path>(
    "mission", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&StatesNode::mission_callback, this, _1));
  pose_sp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "offboard/pose", 10, std::bind(&StatesNode::pose_setpoint_callback, this, _1));
  twist_sp_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "offboard/twist", 10, std::bind(&StatesNode::twist_setpoint_callback, this, _1));

  const auto service = [this](const std::string & name,
    std::function<CommandResult(RobotState &)> action) {
      return create_service<Trigger>(
        name, [this, name, action](
          [[maybe_unused]] const std::shared_ptr<Trigger::Request> request,
          std::shared_ptr<Trigger::Response> response) {command(name, response, action);});
    };
  arming_srv_ = service("arming", [this](RobotState & rs) {
        return machine_.arming(rs, now().seconds());
      });
  takeoff_srv_ = service("takeoff", [this](RobotState & rs) {return machine_.takeoff(rs);});
  landing_srv_ = service("landing", [this](RobotState & rs) {return machine_.landing(rs);});
  loiter_srv_ = service("loiter", [this](RobotState & rs) {return machine_.loiter(rs);});
  offboard_srv_ = service("offboard", [this](RobotState & rs) {
        return machine_.offboard(rs, now().seconds());
      });
  mission_srv_ = service("mission", [this](RobotState & rs) {return machine_.mission(rs);});
  rtl_srv_ = service("rtl", [this](RobotState & rs) {return machine_.rtl(rs);});
}

StatesNode::~StatesNode()
{
}

bool
StatesNode::init(std::shared_ptr<RobotState> robot_state)
{
  robot_state_ = robot_state;
  machine_.reset(*robot_state_);
  return true;
}

bool
StatesNode::cycle(std::shared_ptr<RobotState> robot_state)
{
  machine_.update(*robot_state, now().seconds());
  if (machine_.mission_index() != mission_current_) {
    mission_current_ = machine_.mission_index();
    std_msgs::msg::Int32 msg;
    msg.data = mission_current_;
    mission_current_pub_->publish(msg);
  }
  return true;
}

void
StatesNode::release()
{
  robot_state_.reset();
}

void
StatesNode::command(
  const std::string & name, std::shared_ptr<Trigger::Response> response,
  const std::function<CommandResult(RobotState &)> & action)
{
  const CommandResult result = robot_state_ ?
    action(*robot_state_) : CommandResult{false, name + " rejected: pilotaro is not configured"};
  response->success = result.success;
  response->message = result.message;
  if (result.success) {
    RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", result.message.c_str());
  }
}

void
StatesNode::pose_setpoint_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!offboard_frame_id_.empty() && msg->header.frame_id != offboard_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring offboard/pose in frame '%s', expected '%s'",
      msg->header.frame_id.c_str(), offboard_frame_id_.c_str());
    return;
  }
  const auto & p = msg->pose.position;
  const auto & q = msg->pose.orientation;
  machine_.set_offboard_pose(
    {p.x, p.y, p.z}, yaw_from_quaternion(Eigen::Quaterniond(q.w, q.x, q.y, q.z)),
    now().seconds());
}

void
StatesNode::twist_setpoint_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  machine_.set_offboard_twist(
    {msg->linear.x, msg->linear.y, msg->linear.z}, msg->angular.z, now().seconds());
}

bool
StatesNode::read_failsafe_params(FailsafeParams & f, std::string & error)
{
  const auto num = [this](const char * name) {
      return get_parameter(std::string("failsafe.") + name).as_double();
    };
  const auto act = [this, &error](const char * name, FailsafeAction & a) {
      const auto v = get_parameter(std::string("failsafe.") + name).as_string();
      if (!parse_failsafe_action(v, a)) {
        error = std::string("failsafe.") + name + " = '" + v +
          "': must be none, warn, loiter, land or disarm";
        return false;
      }
      return true;
    };
  f.battery_min_arm = num("battery_min_arm");
  f.battery_low = num("battery_low");
  f.battery_critical = num("battery_critical");
  f.battery_timeout = num("battery_timeout");
  f.geofence_radius = num("geofence_radius");
  f.geofence_max_height = num("geofence_max_height");
  f.geofence_max_depth = num("geofence_max_depth");
  f.geofence_margin = num("geofence_margin");
  f.max_position_std = num("max_position_std");
  f.heartbeat_timeout = num("heartbeat_timeout");
  f.max_tilt = num("max_tilt");
  f.max_tilt_time = num("max_tilt_time");
  f.max_tilt_arming = num("max_tilt_arming");
  f.max_rate_arming = num("max_rate_arming");
  return act("battery_low_action", f.battery_low_action) &&
         act("battery_critical_action", f.battery_critical_action) &&
         act("geofence_action", f.geofence_action) &&
         act("position_action", f.position_action) &&
         act("heartbeat_action", f.heartbeat_action) &&
         act("attitude_action", f.attitude_action);
}

bool
StatesNode::read_mission_params(MissionParams & m, std::string & error)
{
  const auto num = [this](const char * name) {
      return get_parameter(std::string("mission.") + name).as_double();
    };
  const auto act = [this, &error](const char * name, FailsafeAction & a,
      std::initializer_list<FailsafeAction> allowed) {
      const auto v = get_parameter(std::string("mission.") + name).as_string();
      if (parse_failsafe_action(v, a) &&
        std::find(allowed.begin(), allowed.end(), a) != allowed.end())
      {
        return true;
      }
      std::string names;
      for (const auto x : allowed) {
        names += (names.empty() ? "" : ", ") + to_string(x);
      }
      error = std::string("mission.") + name + " = '" + v + "': must be " + names;
      return false;
    };
  m.speed = num("speed");
  m.acceleration = num("acceleration");
  m.acceptance_radius = num("acceptance_radius");
  m.acceptance_speed = num("acceptance_speed");
  m.hold_time = num("hold_time");
  m.max_lead = num("max_lead");
  m.rtl_altitude = num("rtl_altitude");
  m.rtl_speed = num("rtl_speed");
  if (m.speed <= 0.0 || m.rtl_speed <= 0.0 || m.acceptance_radius <= 0.0) {
    error = "mission.speed, mission.rtl_speed and mission.acceptance_radius must be positive";
    return false;
  }
  return act("end_action", m.end_action,
           {FailsafeAction::LOITER, FailsafeAction::RTL, FailsafeAction::LAND}) &&
         act("rtl_final_action", m.rtl_final_action,
           {FailsafeAction::LOITER, FailsafeAction::LAND, FailsafeAction::DISARM});
}

void
StatesNode::mission_callback(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  if (!offboard_frame_id_.empty() && msg->header.frame_id != offboard_frame_id_) {
    RCLCPP_WARN(
      get_logger(), "Ignoring mission in frame '%s', expected '%s'",
      msg->header.frame_id.c_str(), offboard_frame_id_.c_str());
    return;
  }
  std::vector<Waypoint> waypoints;
  for (const auto & pose : msg->poses) {
    Waypoint w;
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;
    w.position = {p.x, p.y, p.z};
    // An unset orientation (all zero) keeps the heading
    if (q.w != 0.0 || q.x != 0.0 || q.y != 0.0 || q.z != 0.0) {
      w.yaw = yaw_from_quaternion(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
    }
    waypoints.push_back(w);
  }
  const auto result = machine_.set_mission(
    waypoints, robot_state_ ? robot_state_->get_mode() : FlightMode::DISARMED);
  if (result.success) {
    RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", result.message.c_str());
  }
}

rcl_interfaces::msg::SetParametersResult
StatesNode::on_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & prm : parameters) {
    if (prm.get_name().rfind("failsafe.", 0) != 0 && prm.get_name().rfind("mission.", 0) != 0) {
      continue;
    }
    if (prm.get_name().size() > 7 &&
      prm.get_name().compare(prm.get_name().size() - 7, 7, "_action") == 0)
    {
      FailsafeAction a;
      if (prm.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        !parse_failsafe_action(prm.as_string(), a))
      {
        result.successful = false;
        result.reason = prm.get_name() + ": must be none, warn, loiter, rtl, land or disarm";
        return result;
      }
    } else if (prm.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
      result.reason = prm.get_name() + ": must be a double";
      return result;
    }
  }
  return result;
}

void
StatesNode::on_failsafe_change(
  const std::vector<std::string> & active, FailsafeAction action)
{
  std::string names;
  for (const auto & a : active) {
    names += (names.empty() ? "" : ",") + a;
  }
  if (names.empty()) {
    RCLCPP_INFO(get_logger(), "Failsafes cleared");
  } else {
    RCLCPP_WARN(
      get_logger(), "Failsafe active: %s (action %s)", names.c_str(), to_string(action).c_str());
  }
  std_msgs::msg::String msg;
  msg.data = names;
  failsafe_pub_->publish(msg);
}

void
StatesNode::on_mode_change(FlightMode from, FlightMode to, const std::string & reason)
{
  if (from != to) {
    RCLCPP_INFO(
      get_logger(), "Mode %s -> %s (%s)", to_string(from).c_str(), to_string(to).c_str(),
      reason.c_str());
  }
  std_msgs::msg::String msg;
  msg.data = to_string(to);
  mode_pub_->publish(msg);
}

}  // namespace pilotaro
