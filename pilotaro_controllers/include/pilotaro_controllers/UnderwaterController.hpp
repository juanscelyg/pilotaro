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

#ifndef PILOTARO_CONTROLLERS__UNDERWATERCONTROLLER_HPP_
#define PILOTARO_CONTROLLERS__UNDERWATERCONTROLLER_HPP_

#include <Eigen/Dense>

#include <array>
#include <string>
#include <vector>

#include "pilotaro_core/ControllerBase.hpp"

namespace pilotaro_controllers
{

using Wrench = Eigen::Matrix<double, 6, 1>;  ///< [Fx, Fy, Fz, Mx, My, Mz], body frame (FLU)

/**
 * @struct UnderwaterGains
 * @brief Gains, limits and hydrodynamic model of the underwater controller.
 */
struct UnderwaterGains
{
  Eigen::Vector3d position_p{1.0, 1.0, 1.0};            ///< [1/s]
  Eigen::Vector3d velocity_p{2.0, 2.0, 2.0};            ///< [1/s]
  Eigen::Vector3d velocity_i{0.3, 0.3, 0.3};            ///< [1/s^2]
  Eigen::Vector3d velocity_i_max{1.0, 1.0, 1.0};        ///< Integral clamp [m/s^2]
  Eigen::Vector3d attitude_p{10.0, 10.0, 6.0};          ///< [1/s^2]
  Eigen::Vector3d angular_rate_p{6.0, 6.0, 5.0};        ///< [1/s]
  double max_horizontal_speed{1.0};                     ///< [m/s]
  double max_vertical_speed{0.5};                       ///< [m/s]
  Eigen::Vector3d added_mass{0.0, 0.0, 0.0};            ///< Surge, sway, heave [kg]
  Eigen::Vector3d added_inertia{0.0, 0.0, 0.0};         ///< Roll, pitch, yaw [kg m^2]
  Eigen::Vector3d linear_damping{0.0, 0.0, 0.0};        ///< Surge, sway, heave [N s/m]
  Eigen::Vector3d quadratic_damping{0.0, 0.0, 0.0};     ///< Surge, sway, heave [N s^2/m^2]
  double net_buoyancy{0.0};                             ///< Buoyancy - weight [N], + floats
};

/**
 * @class UnderwaterController
 * @brief Controller for vehicles with vectored, reversible thrusters (ROVs, AUVs).
 *
 * position (P, world) -> velocity (PI, world) -> force with the mass plus added mass, and
 * feedforward of the damping at the velocity setpoint and of the net buoyancy;
 * attitude: roll and pitch to zero and the yaw of the setpoint, PD on SO(3) with the
 * inertia plus added inertia. The 6 DOF wrench is allocated to the thrusters with the
 * pseudo-inverse of the thruster matrix (position, thrust axis and reaction torque of
 * every thruster) restricted to the DOF the thrusters can control independently, chosen in
 * the order surge, sway, heave, yaw, roll, pitch. The others (e.g. pitch on a BlueROV2 with
 * 6 thrusters, left to its hydrostatic stability) get the wrench that results. If a
 * thruster saturates, all the thrusts are scaled down together, so the direction of the
 * wrench is kept.
 *
 * Parameters (prefix "controller."): position_p, velocity_p, velocity_i, velocity_i_max,
 * attitude_p, angular_rate_p, added_mass, added_inertia, linear_damping, quadratic_damping
 * (3 element vectors), max_horizontal_speed, max_vertical_speed, net_buoyancy.
 */
class UnderwaterController : public pilotaro::ControllerBase
{
public:
  /**
   * @brief Store the parameters and build the thruster matrix
   * @param vehicle physical properties (reversible thrusters with any axis)
   * @param gains controller gains and hydrodynamic model
   * @param[out] error reason when it fails, may be null
   * @return false if the parameters are invalid or the thrusters cannot control surge,
   * sway, heave and yaw
   */
  bool configure(
    const pilotaro::VehicleParams & vehicle, const UnderwaterGains & gains,
    std::string * error = nullptr);

  bool update(
    const pilotaro::VehicleState & state, const pilotaro::ControlSetpoint & setpoint, double dt,
    std::vector<double> & rotor_speeds) override;

  /**
   * @brief Reset the velocity integrator
   */
  void reset() override;

  /**
   * @brief Thruster speeds that produce a body wrench (or its controllable part)
   * @param wrench [Fx, Fy, Fz, Mx, My, Mz] body frame
   * @param[out] rotor_speeds signed angular velocities [rad/s]
   */
  void allocate(const Wrench & wrench, std::vector<double> & rotor_speeds) const;

  /**
   * @brief Body wrench produced by some thruster speeds
   */
  Wrench wrench_from_speeds(const std::vector<double> & rotor_speeds) const;

  /**
   * @brief Whether each of the 6 DOF (x, y, z, roll, pitch, yaw) can be controlled
   */
  const std::array<bool, 6> & controllable() const {return controllable_;}

  /**
   * @brief Physical properties in use
   */
  const pilotaro::VehicleParams & vehicle() const {return vehicle_;}

protected:
  bool on_initialize() override;

private:
  pilotaro::VehicleParams vehicle_;       ///< Physical properties
  UnderwaterGains gains_;       ///< Gains and model
  Eigen::MatrixXd allocation_;       ///< Thrusts -> wrench (6 x N)
  Eigen::MatrixXd allocation_pinv_;       ///< Controlled part of the wrench -> thrusts (N x 6)
  std::array<bool, 6> controllable_{};       ///< DOF the thrusters can produce
  Eigen::Vector3d velocity_integral_{Eigen::Vector3d::Zero()};  ///< Velocity integral [m/s^2]
};

}  // namespace pilotaro_controllers

#endif  // PILOTARO_CONTROLLERS__UNDERWATERCONTROLLER_HPP_
