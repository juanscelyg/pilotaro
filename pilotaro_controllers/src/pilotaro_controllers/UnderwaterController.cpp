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
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "pilotaro_controllers/UnderwaterController.hpp"

namespace pilotaro_controllers
{

namespace
{
Eigen::Vector3d vee(const Eigen::Matrix3d & m)
{
  return {m(2, 1), m(0, 2), m(1, 0)};
}

const char * const kDofNames[6] = {"surge", "sway", "heave", "roll", "pitch", "yaw"};
}  // namespace

bool
UnderwaterController::on_initialize()
{
  UnderwaterGains g;
  g.position_p = declare_vector3("position_p", g.position_p);
  g.velocity_p = declare_vector3("velocity_p", g.velocity_p);
  g.velocity_i = declare_vector3("velocity_i", g.velocity_i);
  g.velocity_i_max = declare_vector3("velocity_i_max", g.velocity_i_max);
  g.attitude_p = declare_vector3("attitude_p", g.attitude_p);
  g.angular_rate_p = declare_vector3("angular_rate_p", g.angular_rate_p);
  g.max_horizontal_speed = declare_parameter("max_horizontal_speed", g.max_horizontal_speed);
  g.max_vertical_speed = declare_parameter("max_vertical_speed", g.max_vertical_speed);
  g.added_mass = declare_vector3("added_mass", g.added_mass);
  g.added_inertia = declare_vector3("added_inertia", g.added_inertia);
  g.linear_damping = declare_vector3("linear_damping", g.linear_damping);
  g.quadratic_damping = declare_vector3("quadratic_damping", g.quadratic_damping);
  g.net_buoyancy = declare_parameter("net_buoyancy", g.net_buoyancy);

  std::string error;
  if (!configure(get_vehicle(), g, &error)) {
    RCLCPP_ERROR(get_logger(), "Invalid configuration: %s", error.c_str());
    return false;
  }
  std::string uncontrolled;
  for (int k = 0; k < 6; ++k) {
    if (!controllable_[k]) {
      uncontrolled += std::string(uncontrolled.empty() ? "" : ", ") + kDofNames[k];
    }
  }
  RCLCPP_INFO(
    get_logger(), "Underwater controller for %zu thrusters (max %.1f N each)%s%s",
    vehicle_.rotor_count(), vehicle_.max_thrust(),
    uncontrolled.empty() ? "" : ", not actuated: ", uncontrolled.c_str());
  return true;
}

bool
UnderwaterController::configure(
  const pilotaro::VehicleParams & vehicle, const UnderwaterGains & gains, std::string * error)
{
  const auto set_error = [error](const std::string & msg) {
      if (error != nullptr) {
        *error = msg;
      }
      return false;
    };
  if (const auto vehicle_error = vehicle.validate(); !vehicle_error.empty()) {
    return set_error("vehicle: " + vehicle_error);
  }
  if (gains.max_horizontal_speed <= 0.0 || gains.max_vertical_speed <= 0.0) {
    return set_error("max speeds must be positive");
  }
  if ((gains.added_mass.array() < 0.0).any() || (gains.added_inertia.array() < 0.0).any() ||
    (gains.linear_damping.array() < 0.0).any() || (gains.quadratic_damping.array() < 0.0).any())
  {
    return set_error("added mass, added inertia and damping must not be negative");
  }
  vehicle_ = vehicle;
  gains_ = gains;

  // Column i: force and torque of a unit thrust of thruster i. The reaction torque of the
  // propeller is opposite to its spin (ccw: negative around the thrust axis).
  const auto n = static_cast<Eigen::Index>(vehicle_.rotor_count());
  allocation_.resize(6, n);
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto axis = vehicle_.rotor_thrust_axis(i);
    allocation_.block<3, 1>(0, i) = axis;
    allocation_.block<3, 1>(3, i) = vehicle_.rotor_position(i).cross(axis) -
      vehicle_.rotor_direction[i] * vehicle_.moment_constant * axis;
  }
  // Controlled DOF: added in order of priority while they are independent of the ones
  // already chosen. With the thrusters off the center of mass, a coupled DOF (e.g. pitch on
  // a 6 thruster BlueROV2, where any surge also pitches) is left out and its wrench is the
  // one that results from the others.
  const int priority[6] = {0, 1, 2, 5, 3, 4};  // surge, sway, heave, yaw, roll, pitch
  std::vector<int> rows;
  controllable_.fill(false);
  for (const int dof : priority) {
    rows.push_back(dof);
    Eigen::MatrixXd selected(rows.size(), n);
    for (std::size_t r = 0; r < rows.size(); ++r) {
      selected.row(static_cast<Eigen::Index>(r)) = allocation_.row(rows[r]);
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu(selected);
    lu.setThreshold(1e-6);
    if (lu.rank() < static_cast<Eigen::Index>(rows.size())) {
      rows.pop_back();
    } else {
      controllable_[dof] = true;
    }
  }
  if (!controllable_[0] || !controllable_[1] || !controllable_[2] || !controllable_[5]) {
    return set_error("the thrusters must at least control surge, sway, heave and yaw");
  }
  // Thrusts from the controlled DOF of the wrench: N x 6, zero columns for the others
  Eigen::MatrixXd selected(rows.size(), n);
  for (std::size_t r = 0; r < rows.size(); ++r) {
    selected.row(static_cast<Eigen::Index>(r)) = allocation_.row(rows[r]);
  }
  const Eigen::MatrixXd selected_pinv =
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(selected).pseudoInverse();
  allocation_pinv_ = Eigen::MatrixXd::Zero(n, 6);
  for (std::size_t r = 0; r < rows.size(); ++r) {
    allocation_pinv_.col(rows[r]) = selected_pinv.col(static_cast<Eigen::Index>(r));
  }
  reset();
  return true;
}

void
UnderwaterController::reset()
{
  velocity_integral_.setZero();
}

void
UnderwaterController::allocate(const Wrench & wrench, std::vector<double> & rotor_speeds) const
{
  const std::size_t n = vehicle_.rotor_count();
  rotor_speeds.resize(n);
  Eigen::VectorXd thrusts = allocation_pinv_ * wrench;
  if (!vehicle_.reversible) {
    thrusts = thrusts.cwiseMax(0.0);
  }
  // Scale every thrust together, so the wrench keeps its direction
  const double peak = thrusts.cwiseAbs().maxCoeff();
  if (peak > vehicle_.max_thrust()) {
    thrusts *= vehicle_.max_thrust() / peak;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const double f = thrusts[static_cast<Eigen::Index>(i)];
    const double w = std::sqrt(std::abs(f) / vehicle_.motor_constant);
    if (vehicle_.reversible) {
      rotor_speeds[i] = std::copysign(std::min(w, vehicle_.max_rot_velocity), f);
    } else {
      rotor_speeds[i] = std::clamp(w, vehicle_.min_rot_velocity, vehicle_.max_rot_velocity);
    }
  }
}

Wrench
UnderwaterController::wrench_from_speeds(const std::vector<double> & rotor_speeds) const
{
  Eigen::VectorXd thrusts(static_cast<Eigen::Index>(rotor_speeds.size()));
  for (std::size_t i = 0; i < rotor_speeds.size(); ++i) {
    thrusts[static_cast<Eigen::Index>(i)] =
      vehicle_.motor_constant * rotor_speeds[i] * std::abs(rotor_speeds[i]);
  }
  return allocation_ * thrusts;
}

bool
UnderwaterController::update(
  const pilotaro::VehicleState & state, const pilotaro::ControlSetpoint & setpoint, double dt,
  std::vector<double> & rotor_speeds)
{
  if (allocation_pinv_.size() == 0 || !(dt > 0.0)) {
    return false;
  }
  const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d rotation = state.orientation.normalized().toRotationMatrix();

  // Position -> velocity (world)
  Eigen::Vector3d velocity_sp = setpoint.velocity;
  const Eigen::Vector3d position_error = setpoint.position - state.position;
  if (setpoint.xy_position) {
    velocity_sp.head<2>() += gains_.position_p.head<2>().cwiseProduct(position_error.head<2>());
  }
  if (setpoint.z_position) {
    velocity_sp.z() += gains_.position_p.z() * position_error.z();
  }
  const double horizontal_speed = velocity_sp.head<2>().norm();
  if (horizontal_speed > gains_.max_horizontal_speed) {
    velocity_sp.head<2>() *= gains_.max_horizontal_speed / horizontal_speed;
  }
  velocity_sp.z() = std::clamp(
    velocity_sp.z(), -gains_.max_vertical_speed, gains_.max_vertical_speed);

  // Velocity -> acceleration (world)
  const Eigen::Vector3d velocity_error = velocity_sp - state.velocity;
  velocity_integral_ += gains_.velocity_i.cwiseProduct(velocity_error) * dt;
  velocity_integral_ = velocity_integral_.cwiseMax(-gains_.velocity_i_max)
    .cwiseMin(gains_.velocity_i_max);
  const Eigen::Vector3d acceleration_sp =
    gains_.velocity_p.cwiseProduct(velocity_error) + velocity_integral_;

  // Force (body): (mass + added mass) * acceleration + damping at the velocity setpoint
  // - net buoyancy
  const Eigen::Vector3d mass = Eigen::Vector3d::Constant(vehicle_.mass) + gains_.added_mass;
  const Eigen::Vector3d velocity_sp_body = rotation.transpose() * velocity_sp;
  const Eigen::Vector3d damping = gains_.linear_damping.cwiseProduct(velocity_sp_body) +
    gains_.quadratic_damping.cwiseProduct(velocity_sp_body.cwiseProduct(
      velocity_sp_body.cwiseAbs()));
  const Eigen::Vector3d force = mass.cwiseProduct(rotation.transpose() * acceleration_sp) +
    damping - rotation.transpose() * (gains_.net_buoyancy * e3);

  // Attitude: level, with the yaw of the setpoint
  const Eigen::Matrix3d rotation_sp =
    Eigen::AngleAxisd(setpoint.yaw, e3).toRotationMatrix();
  const Eigen::Vector3d attitude_error =
    0.5 * vee(rotation_sp.transpose() * rotation - rotation.transpose() * rotation_sp);
  const Eigen::Vector3d & w = state.angular_velocity;
  const Eigen::Vector3d inertia = vehicle_.inertia + gains_.added_inertia;
  const Eigen::Vector3d torque = inertia.cwiseProduct(
    -gains_.attitude_p.cwiseProduct(attitude_error) - gains_.angular_rate_p.cwiseProduct(w)) +
    w.cross(inertia.cwiseProduct(w));

  Wrench wrench;
  wrench << force, torque;
  for (int k = 0; k < 6; ++k) {
    if (!controllable_[k]) {
      wrench[k] = 0.0;
    }
  }
  allocate(wrench, rotor_speeds);
  return true;
}

}  // namespace pilotaro_controllers

PLUGINLIB_EXPORT_CLASS(pilotaro_controllers::UnderwaterController, pilotaro::ControllerBase)
