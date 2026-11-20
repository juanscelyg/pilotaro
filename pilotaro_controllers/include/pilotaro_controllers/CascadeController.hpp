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

#ifndef PILOTARO_CONTROLLERS__CASCADECONTROLLER_HPP_
#define PILOTARO_CONTROLLERS__CASCADECONTROLLER_HPP_

#include <Eigen/Dense>

#include <string>
#include <vector>

#include "pilotaro_core/ControllerBase.hpp"
#include "pilotaro_controllers/Shaping.hpp"

namespace pilotaro_controllers
{

/**
 * @struct CascadeGains
 * @brief Gains and limits of the cascaded controller.
 */
struct CascadeGains
{
  Eigen::Vector3d position_p{1.0, 1.0, 1.5};            ///< [1/s]
  Eigen::Vector3d velocity_p{2.5, 2.5, 4.0};            ///< [1/s]
  Eigen::Vector3d velocity_i{0.5, 0.5, 1.0};            ///< [1/s^2]
  Eigen::Vector3d velocity_i_max{2.0, 2.0, 3.0};        ///< Integral clamp [m/s^2]
  Eigen::Vector3d attitude_p{60.0, 60.0, 20.0};         ///< [1/s^2]
  Eigen::Vector3d angular_rate_p{14.0, 14.0, 8.0};      ///< [1/s]
  double max_horizontal_speed{3.0};                     ///< [m/s]
  double max_vertical_speed{1.5};                       ///< [m/s]
  double max_tilt{0.6};                                 ///< [rad]
  double max_yaw_rate{1.0};          ///< Yaw rate commanded by the yaw error [rad/s], 0 = none
  double position_cutoff{5.0};       ///< Low-pass of the measured position [Hz], 0 = none
  double velocity_cutoff{10.0};      ///< Low-pass of the measured velocity [Hz], 0 = none
  double angular_rate_cutoff{30.0};  ///< Low-pass of the measured angular velocity [Hz]
};

/**
 * @class CascadeController
 * @brief Cascaded multirotor controller.
 *
 * position (P) -> velocity (PI) -> thrust vector -> attitude on SO(3) (PD)
 * -> body torques -> motor allocation -> rotor angular velocities.
 *
 * The allocation uses the pseudo-inverse of the [thrust, Mx, My, Mz] matrix, so any
 * planar geometry with 4 or more rotors works (equal to the inverse for a quadrotor).
 * When the rotors cannot give the whole wrench it is desaturated by priorities: roll and
 * pitch first (moving the collective thrust if needed, scaled down only if they do not fit),
 * then as much yaw torque as fits in the remaining margin. A yaw demand never takes thrust
 * or attitude authority away.
 *
 * The yaw error commands a yaw rate limited to max_yaw_rate (P -> rate -> torque, the same
 * as the PD law while not limited), so a large yaw step does not ask for more torque than
 * the rotors have.
 *
 * The measured position, velocity and angular velocity are low-pass filtered
 * (position_cutoff, velocity_cutoff, angular_rate_cutoff), so estimation noise does not
 * reach the motors. With a 3D position setpoint the velocity setpoint points straight at
 * the goal (position_to_velocity).
 *
 * Parameters (prefix "controller."): position_p, velocity_p, velocity_i, velocity_i_max,
 * attitude_p, angular_rate_p (3 element vectors), max_horizontal_speed,
 * max_vertical_speed, max_tilt, max_yaw_rate, position_cutoff, velocity_cutoff, angular_rate_cutoff.
 */
class CascadeController : public pilotaro::ControllerBase
{
public:
  /**
   * @brief Store the parameters and build the allocation matrix
   * @param vehicle physical properties
   * @param gains controller gains
   * @param[out] error reason when it fails, may be null
   * @return false if the parameters are invalid or the rotor geometry cannot produce
   * independent thrust and torques
   */
  bool configure(
    const pilotaro::VehicleParams & vehicle, const CascadeGains & gains,
    std::string * error = nullptr);

  bool update(
    const pilotaro::VehicleState & state, const pilotaro::ControlSetpoint & setpoint, double dt,
    std::vector<double> & rotor_speeds) override;

  /**
   * @brief Reset the velocity integrator
   */
  void reset() override;

  /**
   * @brief Convert a collective thrust and body torques to rotor speeds, desaturating by
   * priorities (roll and pitch, then thrust, then yaw) when the rotors cannot give it all
   * @param wrench [thrust, Mx, My, Mz]
   * @param[out] rotor_speeds rotor angular velocities [rad/s]
   */
  void allocate(const Eigen::Vector4d & wrench, std::vector<double> & rotor_speeds) const;

  /**
   * @brief Physical properties in use
   */
  const pilotaro::VehicleParams & vehicle() const {return vehicle_;}

protected:
  bool on_initialize() override;

private:
  pilotaro::VehicleParams vehicle_;       ///< Physical properties
  CascadeGains gains_;       ///< Gains and limits
  Eigen::MatrixXd allocation_pinv_;       ///< wrench -> rotor thrusts (N x 4)
  Eigen::Vector3d velocity_integral_{Eigen::Vector3d::Zero()};  ///< Velocity integral [m/s^2]
  LowPass3 position_filter_;       ///< Measured position
  LowPass3 velocity_filter_;       ///< Measured velocity
  LowPass3 rate_filter_;       ///< Measured angular velocity
};

}  // namespace pilotaro_controllers

#endif  // PILOTARO_CONTROLLERS__CASCADECONTROLLER_HPP_
