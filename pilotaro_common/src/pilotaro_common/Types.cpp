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

#include <cmath>
#include <string>

#include "pilotaro_common/Types.hpp"

namespace pilotaro
{

std::string to_string(FlightMode mode)
{
  switch (mode) {
    case FlightMode::DISARMED: return "DISARMED";
    case FlightMode::ARMED: return "ARMED";
    case FlightMode::TAKEOFF: return "TAKEOFF";
    case FlightMode::LOITER: return "LOITER";
    case FlightMode::LANDING: return "LANDING";
    case FlightMode::OFFBOARD: return "OFFBOARD";
  }
  return "UNKNOWN";
}

bool is_flying(FlightMode mode)
{
  return mode == FlightMode::TAKEOFF || mode == FlightMode::LOITER ||
         mode == FlightMode::LANDING || mode == FlightMode::OFFBOARD;
}

std::string
VehicleParams::validate() const
{
  if (rotor_x.size() < 4) {
    return "at least 4 rotors are needed, got " + std::to_string(rotor_x.size());
  }
  if (rotor_y.size() != rotor_x.size() || rotor_direction.size() != rotor_x.size()) {
    return "rotor_x, rotor_y and rotor_direction must have the same size";
  }
  if (!rotor_z.empty() && rotor_z.size() != rotor_x.size()) {
    return "rotor_z must be empty or have one value per rotor";
  }
  if (!rotor_axis.empty() && rotor_axis.size() != 3 * rotor_x.size()) {
    return "rotor_axis must be empty or have 3 values per rotor";
  }
  for (std::size_t i = 0; i < rotor_count(); ++i) {
    if (!rotor_axis.empty() &&
      Eigen::Vector3d(rotor_axis[3 * i], rotor_axis[3 * i + 1], rotor_axis[3 * i + 2]).norm() <
      1e-6)
    {
      return "rotor_axis of rotor " + std::to_string(i) + " is zero";
    }
  }
  for (const double d : rotor_direction) {
    if (d != 1.0 && d != -1.0) {
      return "rotor_direction must be 1 (ccw) or -1 (cw)";
    }
  }
  if (mass <= 0.0 || gravity <= 0.0 || (inertia.array() <= 0.0).any()) {
    return "mass, gravity and inertia must be positive";
  }
  if (motor_constant <= 0.0 || moment_constant < 0.0) {
    return "motor_constant must be positive and moment_constant not negative";
  }
  if (min_rot_velocity < 0.0 || max_rot_velocity <= min_rot_velocity) {
    return "0 <= min_rot_velocity < max_rot_velocity is required";
  }
  return "";
}

Eigen::Vector3d
VehicleParams::rotor_position(std::size_t i) const
{
  return {rotor_x[i], rotor_y[i], rotor_z.empty() ? 0.0 : rotor_z[i]};
}

Eigen::Vector3d
VehicleParams::rotor_thrust_axis(std::size_t i) const
{
  if (rotor_axis.empty()) {
    return Eigen::Vector3d::UnitZ();
  }
  return Eigen::Vector3d(rotor_axis[3 * i], rotor_axis[3 * i + 1], rotor_axis[3 * i + 2])
         .normalized();
}

bool
VehicleParams::vertical_rotors() const
{
  for (std::size_t i = 0; i < rotor_count(); ++i) {
    if (rotor_thrust_axis(i).z() < 1.0 - 1e-6) {
      return false;
    }
  }
  return true;
}

double
VehicleParams::hover_rot_velocity() const
{
  return std::sqrt(mass * gravity / (static_cast<double>(rotor_count()) * motor_constant));
}

}  // namespace pilotaro
