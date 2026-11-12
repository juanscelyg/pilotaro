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

#ifndef PILOTARO_STATES__FLIGHTMODEMACHINE_HPP_
#define PILOTARO_STATES__FLIGHTMODEMACHINE_HPP_

#include <Eigen/Dense>

#include <cstdint>
#include <functional>
#include <string>
#include <utility>

#include "pilotaro_common/RobotState.hpp"

namespace pilotaro
{

/**
 * @struct FlightModeParams
 * @brief Parameters of the flight state machine.
 */
struct FlightModeParams
{
  /// Takeoff height over the arming point [m]; negative dives (underwater vehicles)
  double takeoff_altitude{2.0};
  double takeoff_speed{0.8};       ///< Climb speed in TAKEOFF [m/s]
  double landing_speed{0.5};       ///< Descent speed in LANDING [m/s]
  double land_detect_altitude{0.5};       ///< Max height over home to detect touchdown [m]
  double land_detect_speed{0.1};       ///< Max vertical speed to detect touchdown [m/s]
  double land_detect_time{1.0};       ///< Time the conditions must hold [s]
  double offboard_timeout{0.5};       ///< Max age of an offboard setpoint [s]
  double estimate_timeout{0.5};       ///< Max age of the estimate while armed [s]
  double min_dt{1e-4};       ///< Lower bound of the time step [s]
  double max_dt{0.05};       ///< Upper bound of the time step [s]
};

/**
 * @struct CommandResult
 * @brief Answer to a mode command.
 */
struct CommandResult
{
  bool success;
  std::string message;
};

/**
 * @class FlightModeMachine
 * @brief Flight modes of the ACKS autopilot, without ROS.
 *
 * DISARMED --arming--> ARMED --takeoff--> TAKEOFF --(altitude reached)--> LOITER
 * ARMED --arming--> DISARMED
 * any but DISARMED --offboard (fresh setpoint)--> OFFBOARD --(setpoint timeout)--> LOITER
 * flying --loiter--> LOITER, flying --landing--> LANDING --(touchdown)--> DISARMED
 * armed --(estimate timeout)--> DISARMED
 *
 * It reads the estimate and writes the mode, the setpoint and the controller reset
 * request of the robot state. Times are in seconds, in the clock of the nodes.
 */
class FlightModeMachine
{
public:
  using ModeCallback = std::function<void(FlightMode from, FlightMode to,
      const std::string & reason)>;

  /**
   * @brief Set the parameters
   */
  void configure(const FlightModeParams & params) {params_ = params;}

  /**
   * @brief Called on every mode change (also when the mode is set again)
   */
  void set_mode_callback(ModeCallback callback) {mode_callback_ = std::move(callback);}

  /**
   * @brief Go to DISARMED and forget the offboard setpoints
   */
  void reset(RobotState & robot_state);

  /**
   * @brief Arm (DISARMED) or disarm (ARMED, on the ground)
   */
  CommandResult arming(RobotState & robot_state, double now);

  /**
   * @brief Climb (or dive, if negative) takeoff_altitude from the current point (ARMED)
   */
  CommandResult takeoff(RobotState & robot_state);

  /**
   * @brief Descend at landing_speed keeping XY, disarm on touchdown (flying)
   */
  CommandResult landing(RobotState & robot_state);

  /**
   * @brief Hold the current position and heading (flying)
   */
  CommandResult loiter(RobotState & robot_state);

  /**
   * @brief Follow the offboard setpoints (any mode but DISARMED, needs a fresh setpoint)
   */
  CommandResult offboard(RobotState & robot_state, double now);

  /**
   * @brief Store an offboard position setpoint
   * @param position world frame [m]
   * @param yaw heading [rad]
   * @param now reception time [s]
   */
  void set_offboard_pose(const Eigen::Vector3d & position, double yaw, double now);

  /**
   * @brief Store an offboard velocity setpoint
   * @param velocity world frame [m/s]
   * @param yaw_rate [rad/s]
   * @param now reception time [s]
   */
  void set_offboard_twist(const Eigen::Vector3d & velocity, double yaw_rate, double now);

  /**
   * @brief Check the estimate timeout and, on a new estimate, advance the setpoint of the
   * current mode and run the automatic transitions
   */
  void update(RobotState & robot_state, double now);

private:
  /**
   * @brief Change the flight mode and notify it
   */
  void set_mode(RobotState & robot_state, FlightMode mode, const std::string & reason);

  /**
   * @brief Set the setpoint to the current position and heading
   */
  void hold_current_position(RobotState & robot_state);

  /**
   * @brief Whether a fresh offboard setpoint is available
   */
  bool offboard_setpoint_valid(double now) const;

  /**
   * @brief Advance the setpoint according to the current flight mode
   */
  void update_setpoint(RobotState & robot_state, double dt, double now);

  FlightModeParams params_;       ///< Parameters
  ModeCallback mode_callback_;       ///< Mode change notification

  enum class OffboardType {NONE, POSE, TWIST};
  OffboardType offboard_type_{OffboardType::NONE};       ///< Type of the last offboard setpoint
  Eigen::Vector3d offboard_position_{Eigen::Vector3d::Zero()};       ///< Last pose setpoint
  double offboard_yaw_{0.0};       ///< Yaw of the last pose setpoint [rad]
  Eigen::Vector3d offboard_velocity_{Eigen::Vector3d::Zero()};       ///< Last twist setpoint
  double offboard_yaw_rate_{0.0};       ///< Yaw rate of the last twist setpoint [rad/s]
  double offboard_stamp_{0.0};       ///< Reception time of the last offboard setpoint [s]

  double home_z_{0.0};       ///< Altitude when armed [m]
  double takeoff_target_z_{0.0};       ///< Altitude to reach in TAKEOFF [m]
  double landed_time_{0.0};       ///< Time the land detector has been true [s]

  uint64_t last_seq_{0};       ///< Last estimate processed
  double last_stamp_{0.0};       ///< Stamp of that estimate [s]
  bool has_last_{false};       ///< An estimate was processed
};

/**
 * @brief Yaw of a quaternion
 */
double yaw_from_quaternion(const Eigen::Quaterniond & q);

}  // namespace pilotaro

#endif  // PILOTARO_STATES__FLIGHTMODEMACHINE_HPP_
