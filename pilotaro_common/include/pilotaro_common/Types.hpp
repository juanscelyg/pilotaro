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

#ifndef PILOTARO_COMMON__TYPES_HPP_
#define PILOTARO_COMMON__TYPES_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <vector>

namespace pilotaro
{

/**
 * @enum FlightMode
 * @brief Modes of the flight state machine.
 */
enum class FlightMode
{
  DISARMED,       ///< Motors stopped
  ARMED,          ///< On ground, motors at idle
  TAKEOFF,        ///< Climbing to the takeoff altitude
  LOITER,         ///< Holding position
  LANDING,        ///< Descending until touchdown, then disarm
  OFFBOARD,       ///< Following external setpoints
};

/**
 * @brief Name of a flight mode
 * @param mode flight mode
 * @return upper case name
 */
std::string to_string(FlightMode mode);

/**
 * @brief Whether the vehicle is in the air or about to leave the ground in this mode
 * @param mode flight mode
 * @return true for TAKEOFF, LOITER, LANDING and OFFBOARD
 */
bool is_flying(FlightMode mode);

/**
 * @struct VehicleState
 * @brief Estimated state. Position and velocity in world frame (ENU),
 * angular velocity in body frame (FLU).
 */
struct VehicleState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  double stamp{0.0};       ///< Time of the estimate [s], clock of the estimator
};

/**
 * @struct ControlSetpoint
 * @brief Reference for the controller, expressed in world frame (ENU).
 *
 * When an axis is in position mode, velocity acts as feedforward.
 * Otherwise only the velocity is tracked on that axis.
 */
struct ControlSetpoint
{
  bool xy_position{true};       ///< Track horizontal position (true) or horizontal velocity (false)
  bool z_position{true};        ///< Track altitude (true) or vertical velocity (false)
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double yaw{0.0};              ///< Heading [rad], ENU (0 = east, CCW positive)
};

/**
 * @struct VehicleParams
 * @brief Physical properties of the vehicle, shared by every plugin.
 *
 * A vehicle has N rotors (propellers or thrusters). Rotor i is at (rotor_x, rotor_y, rotor_z)
 * from the center of mass and pushes along rotor_axis (body frame, FLU) with
 * thrust = motor_constant * w * |w|; it reacts on the body with a torque of
 * -rotor_direction * moment_constant * thrust around that axis. Multirotor rotors only
 * spin forward (w in [min_rot_velocity, max_rot_velocity]); reversible thrusters spin both
 * ways (|w| <= max_rot_velocity). rotor_z and rotor_axis may be empty: rotors in the plane
 * of the center of mass pushing up (+z).
 *
 * The defaults are the x500 of pilotaro_description. Run
 * pilotaro_description/scripts/update_vehicle_params.py to regenerate them from the URDF.
 */
struct VehicleParams
{
  double mass{2.11431};                                 ///< [kg]
  double gravity{9.8};                                  ///< [m/s^2]
  Eigen::Vector3d inertia{0.0238949, 0.0244645, 0.0444875};   ///< Diagonal, about the CoM [kg m^2]
  /// [m] from the CoM (FLU)
  std::vector<double> rotor_x{0.171872, -0.176128, 0.171872, -0.176128};
  std::vector<double> rotor_y{-0.174, 0.174, 0.174, -0.174};   ///< [m] from the CoM (FLU)
  std::vector<double> rotor_z;                          ///< [m] from the CoM (FLU), empty: 0
  /// Thrust direction, body frame (FLU), 3 values per rotor. Empty: +z for every rotor
  std::vector<double> rotor_axis;
  std::vector<double> rotor_direction{1.0, 1.0, -1.0, -1.0};   ///< 1 = ccw, -1 = cw
  double motor_constant{8.54858e-06};                   ///< thrust = motor_constant * w * |w|
  double moment_constant{0.016};                        ///< drag torque = moment_constant * thrust
  double min_rot_velocity{100.0};                       ///< [rad/s] while flying
  double max_rot_velocity{1000.0};                      ///< [rad/s]
  bool reversible{false};                               ///< Rotors can spin (and push) both ways

  /**
   * @brief Number of rotors
   */
  std::size_t rotor_count() const {return rotor_x.size();}

  /**
   * @brief Position of a rotor from the center of mass, body frame
   */
  Eigen::Vector3d rotor_position(std::size_t i) const;

  /**
   * @brief Unit thrust direction of a rotor, body frame
   */
  Eigen::Vector3d rotor_thrust_axis(std::size_t i) const;

  /**
   * @brief Whether every rotor pushes along +z (a multirotor)
   */
  bool vertical_rotors() const;

  /**
   * @brief Thrust of a rotor at its maximum speed [N]
   */
  double max_thrust() const {return motor_constant * max_rot_velocity * max_rot_velocity;}

  /**
   * @brief Check the parameters
   * @return empty if valid, the reason otherwise
   */
  std::string validate() const;

  /**
   * @brief Rotor speed needed to hover with all the rotors equal
   * @return angular velocity [rad/s]
   */
  double hover_rot_velocity() const;
};

}  // namespace pilotaro

#endif  // PILOTARO_COMMON__TYPES_HPP_
