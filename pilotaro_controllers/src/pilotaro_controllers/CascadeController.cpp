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

#include "pilotaro_controllers/CascadeController.hpp"

namespace pilotaro_controllers
{

namespace
{
Eigen::Vector3d vee(const Eigen::Matrix3d & m)
{
  return {m(2, 1), m(0, 2), m(1, 0)};
}
}  // namespace

bool
CascadeController::on_initialize()
{
  CascadeGains g;
  g.position_p = declare_vector3("position_p", g.position_p);
  g.velocity_p = declare_vector3("velocity_p", g.velocity_p);
  g.velocity_i = declare_vector3("velocity_i", g.velocity_i);
  g.velocity_i_max = declare_vector3("velocity_i_max", g.velocity_i_max);
  g.attitude_p = declare_vector3("attitude_p", g.attitude_p);
  g.angular_rate_p = declare_vector3("angular_rate_p", g.angular_rate_p);
  g.max_horizontal_speed = declare_parameter("max_horizontal_speed", g.max_horizontal_speed);
  g.max_vertical_speed = declare_parameter("max_vertical_speed", g.max_vertical_speed);
  g.max_tilt = declare_parameter("max_tilt", g.max_tilt);

  std::string error;
  if (!configure(get_vehicle(), g, &error)) {
    RCLCPP_ERROR(get_logger(), "Invalid configuration: %s", error.c_str());
    return false;
  }
  RCLCPP_INFO(
    get_logger(), "Cascade controller for %zu rotors (hover rotor speed %.1f rad/s)",
    vehicle_.rotor_count(), vehicle_.hover_rot_velocity());
  return true;
}

bool
CascadeController::configure(
  const pilotaro::VehicleParams & vehicle, const CascadeGains & gains, std::string * error)
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
  if (!vehicle.vertical_rotors() || vehicle.reversible) {
    return set_error(
      "the cascade controller is for multirotors: every rotor must push along +z and only "
      "forward (use UnderwaterController for vectored or reversible thrusters)");
  }
  if (gains.max_horizontal_speed <= 0.0 || gains.max_vertical_speed <= 0.0 ||
    gains.max_tilt <= 0.0 || gains.max_tilt >= M_PI_2)
  {
    return set_error("max speeds must be positive and 0 < max_tilt < pi/2");
  }
  vehicle_ = vehicle;
  gains_ = gains;

  // Rows: thrust, roll torque, pitch torque, yaw torque produced by each rotor thrust.
  // A ccw rotor reacts on the body with a negative yaw torque.
  const auto n = static_cast<Eigen::Index>(vehicle_.rotor_count());
  Eigen::MatrixXd allocation(4, n);
  for (Eigen::Index i = 0; i < n; ++i) {
    allocation(0, i) = 1.0;
    allocation(1, i) = vehicle_.rotor_y[i];
    allocation(2, i) = -vehicle_.rotor_x[i];
    allocation(3, i) = -vehicle_.rotor_direction[i] * vehicle_.moment_constant;
  }
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(allocation);
  if (cod.rank() < 4) {
    return set_error("the rotor geometry gives a singular allocation matrix");
  }
  allocation_pinv_ = cod.pseudoInverse();
  thrusts_ = Eigen::VectorXd::Zero(n);
  reset();
  return true;
}

void
CascadeController::reset()
{
  velocity_integral_.setZero();
}

void
CascadeController::allocate(
  const Eigen::Vector4d & wrench, std::vector<double> & rotor_speeds) const
{
  rotor_speeds.resize(vehicle_.rotor_count());
  const Eigen::VectorXd thrusts = allocation_pinv_ * wrench;
  for (std::size_t i = 0; i < rotor_speeds.size(); ++i) {
    const double w = std::sqrt(
      std::max(thrusts[static_cast<Eigen::Index>(i)], 0.0) / vehicle_.motor_constant);
    rotor_speeds[i] = std::clamp(w, vehicle_.min_rot_velocity, vehicle_.max_rot_velocity);
  }
}

bool
CascadeController::update(
  const pilotaro::VehicleState & state, const pilotaro::ControlSetpoint & setpoint, double dt,
  std::vector<double> & rotor_speeds)
{
  if (allocation_pinv_.size() == 0 || !(dt > 0.0)) {
    return false;
  }
  const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();

  // Position -> velocity
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

  // Velocity -> acceleration
  const Eigen::Vector3d velocity_error = velocity_sp - state.velocity;
  velocity_integral_ += gains_.velocity_i.cwiseProduct(velocity_error) * dt;
  velocity_integral_ = velocity_integral_.cwiseMax(-gains_.velocity_i_max)
    .cwiseMin(gains_.velocity_i_max);
  const Eigen::Vector3d acceleration_sp =
    gains_.velocity_p.cwiseProduct(velocity_error) + velocity_integral_;

  // Acceleration -> thrust vector, limiting the tilt
  Eigen::Vector3d force = vehicle_.mass * (acceleration_sp + vehicle_.gravity * e3);
  force.z() = std::max(force.z(), 0.1 * vehicle_.mass * vehicle_.gravity);
  const double max_horizontal_force = force.z() * std::tan(gains_.max_tilt);
  const double horizontal_force = force.head<2>().norm();
  if (horizontal_force > max_horizontal_force) {
    force.head<2>() *= max_horizontal_force / horizontal_force;
  }

  // Thrust vector + yaw -> desired attitude
  const Eigen::Vector3d b3 = force.normalized();
  const Eigen::Vector3d b1c(std::cos(setpoint.yaw), std::sin(setpoint.yaw), 0.0);
  const Eigen::Vector3d b2 = b3.cross(b1c).normalized();
  Eigen::Matrix3d rotation_sp;
  rotation_sp << b2.cross(b3), b2, b3;

  const Eigen::Matrix3d rotation = state.orientation.normalized().toRotationMatrix();
  const double thrust = force.dot(rotation.col(2));

  // Attitude error on SO(3) -> body torques
  const Eigen::Vector3d attitude_error =
    0.5 * vee(rotation_sp.transpose() * rotation - rotation.transpose() * rotation_sp);
  const Eigen::Vector3d & w = state.angular_velocity;
  const Eigen::Vector3d inertia_w = vehicle_.inertia.cwiseProduct(w);
  const Eigen::Vector3d torque = vehicle_.inertia.cwiseProduct(
    -gains_.attitude_p.cwiseProduct(attitude_error) - gains_.angular_rate_p.cwiseProduct(w)) +
    w.cross(inertia_w);

  Eigen::Vector4d wrench;
  wrench << std::max(thrust, 0.0), torque;
  allocate(wrench, rotor_speeds);
  return true;
}

}  // namespace pilotaro_controllers

PLUGINLIB_EXPORT_CLASS(pilotaro_controllers::CascadeController, pilotaro::ControllerBase)
