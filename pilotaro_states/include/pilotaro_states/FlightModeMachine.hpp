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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "pilotaro_common/RobotState.hpp"

namespace pilotaro
{

/**
 * @brief Reaction to a failsafe, from the mildest: only report it, hold the position, return
 * to the arming point, land (an underwater vehicle goes down to the bottom) or stop the
 * motors (a positively buoyant underwater vehicle then surfaces by itself)
 */
enum class FailsafeAction {NONE, WARN, LOITER, RTL, LAND, DISARM};

/// Name of a failsafe action ("none", "warn", "loiter", "rtl", "land", "disarm")
std::string to_string(FailsafeAction action);

/// Parse an action name; false if unknown
bool parse_failsafe_action(const std::string & name, FailsafeAction & action);

/**
 * @struct FailsafeParams
 * @brief Failsafes and pre-arm checks. A threshold of 0 disables its check.
 */
struct FailsafeParams
{
  // Battery (remaining charge 0-1, from set_battery()); unused until a battery is reported
  double battery_min_arm{0.3};       ///< Min charge to arm
  double battery_low{0.2};       ///< Low charge
  FailsafeAction battery_low_action{FailsafeAction::WARN};
  double battery_critical{0.1};       ///< Critical charge
  FailsafeAction battery_critical_action{FailsafeAction::LAND};
  double battery_timeout{2.0};       ///< No battery report for this long: warning [s]

  // Geofence around the arming point (home)
  double geofence_radius{0.0};       ///< Max horizontal distance [m]
  double geofence_max_height{0.0};       ///< Max height over home [m]
  double geofence_max_depth{0.0};       ///< Max depth under home [m]
  /// The setpoints are clamped to the fence; the failsafe triggers beyond it by more than
  /// this (tracking error) [m]
  double geofence_margin{0.3};
  FailsafeAction geofence_action{FailsafeAction::LOITER};

  // Navigation: horizontal position uncertainty of the estimate (VehicleState::position_std)
  double max_position_std{1.0};       ///< [m]
  FailsafeAction position_action{FailsafeAction::LAND};

  // Ground station link (set_heartbeat())
  double heartbeat_timeout{0.0};       ///< [s]
  FailsafeAction heartbeat_action{FailsafeAction::LOITER};

  // Attitude failure (flip, lost motor)
  double max_tilt{1.2};       ///< [rad]
  double max_tilt_time{0.3};       ///< Time beyond max_tilt to trigger [s]
  FailsafeAction attitude_action{FailsafeAction::DISARM};

  // Pre-arm checks
  double max_tilt_arming{0.35};       ///< [rad]
  double max_rate_arming{0.3};       ///< Angular velocity [rad/s]
};

/**
 * @struct MissionParams
 * @brief Waypoint missions and return to launch.
 *
 * The setpoint travels each leg in a straight line at speed, accelerating and braking at
 * acceleration, never more than max_lead ahead of the vehicle.
 */
struct MissionParams
{
  double speed{2.0};       ///< Cruise speed [m/s]
  double acceleration{1.0};       ///< Speed up and brake [m/s^2]
  double acceptance_radius{0.3};       ///< A waypoint is reached closer than this [m]...
  double acceptance_speed{0.5};       ///< ...and slower than this [m/s] (0 = any speed)
  double hold_time{0.0};       ///< Time at each waypoint [s]
  double max_lead{1.0};       ///< Max distance of the setpoint ahead of the vehicle [m]
  /// After the last waypoint: LOITER, RTL or LAND
  FailsafeAction end_action{FailsafeAction::LOITER};
  /// RTL height over the arming point [m]: it climbs to it first if lower (never descends);
  /// 0 = return at the current height
  double rtl_altitude{5.0};
  double rtl_speed{2.0};       ///< [m/s]
  /// Over the arming point: LAND, DISARM (an underwater vehicle surfaces) or LOITER
  FailsafeAction rtl_final_action{FailsafeAction::LAND};
};

/**
 * @struct Waypoint
 * @brief Mission waypoint in the world frame.
 */
struct Waypoint
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};       ///< [m]
  double yaw{std::nan("")};       ///< Heading [rad]; NaN keeps the current one
};

/**
 * @struct FlightModeParams
 * @brief Parameters of the flight state machine.
 */
struct FlightModeParams
{
  /// Takeoff height over the arming point [m]; negative dives (underwater vehicles)
  double takeoff_altitude{2.0};
  double takeoff_speed{0.8};       ///< Climb speed in TAKEOFF [m/s]
  /// Acceleration and braking of the takeoff setpoint [m/s^2]; 0 = constant speed ramp
  double takeoff_acceleration{0.5};
  /// Max distance the takeoff setpoint may lead the estimate [m] (no windup on the ground);
  /// 0 = unlimited
  double takeoff_max_lead{0.3};
  double landing_speed{0.5};       ///< Descent speed in LANDING [m/s]
  double land_detect_altitude{0.5};       ///< Max height over home to detect touchdown [m]
  double land_detect_speed{0.1};       ///< Max vertical speed to detect touchdown [m/s]
  double land_detect_time{1.0};       ///< Time the conditions must hold [s]
  double offboard_timeout{0.5};       ///< Max age of an offboard setpoint [s]
  double estimate_timeout{0.5};       ///< Max age of the estimate while armed [s]
  double min_dt{1e-4};       ///< Lower bound of the time step [s]
  double max_dt{0.05};       ///< Upper bound of the time step [s]
  FailsafeParams failsafe;       ///< Failsafes and pre-arm checks
  MissionParams mission;       ///< Missions and return to launch
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
 * @brief Flight modes of pilotaro, without ROS.
 *
 * DISARMED --arming--> ARMED --takeoff--> TAKEOFF --(altitude reached)--> LOITER
 * ARMED --arming--> DISARMED
 * any but DISARMED --offboard (fresh setpoint)--> OFFBOARD --(setpoint timeout)--> LOITER
 * flying --loiter--> LOITER, flying --landing--> LANDING --(touchdown)--> DISARMED
 * ARMED --mission--> TAKEOFF --(altitude reached)--> MISSION; flying --mission--> MISSION
 * MISSION --(last waypoint)--> LOITER, RTL or LANDING (mission.end_action)
 * flying --rtl--> RTL --(over home)--> LANDING, DISARMED or LOITER (mission.rtl_final_action)
 * armed --(estimate timeout)--> DISARMED
 *
 * Failsafes (FailsafeParams): battery low and critical, geofence, navigation (position
 * uncertainty), ground station link and attitude failure. Each one has an action (warn,
 * loiter, land, disarm); the strongest active action is applied, and while it is active
 * the commands that would undo it (takeoff, offboard) are rejected. Arming runs pre-arm
 * checks (estimate, attitude, battery, link, navigation).
 *
 * It reads the estimate and writes the mode, the setpoint and the controller reset
 * request of the robot state. Times are in seconds, in the clock of the nodes.
 */
class FlightModeMachine
{
public:
  using ModeCallback = std::function<void(FlightMode from, FlightMode to,
      const std::string & reason)>;
  using FailsafeCallback = std::function<void(const std::vector<std::string> & active,
      FailsafeAction action)>;

  /**
   * @brief Set the parameters
   */
  void configure(const FlightModeParams & params) {params_ = params;}

  /**
   * @brief Called on every mode change (also when the mode is set again)
   */
  void set_mode_callback(ModeCallback callback) {mode_callback_ = std::move(callback);}

  /**
   * @brief Called when the set of active failsafes changes
   */
  void set_failsafe_callback(FailsafeCallback callback)
  {
    failsafe_callback_ = std::move(callback);
  }

  /// Parameters in use
  const FlightModeParams & params() const {return params_;}

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
   * @brief Store a mission (replaces the previous one; rejected while flying one)
   */
  CommandResult set_mission(const std::vector<Waypoint> & waypoints, FlightMode mode);

  /**
   * @brief Fly the stored mission: from ARMED it takes off first; flying, it starts from
   * the current position. Rejected if a waypoint is outside the geofence
   */
  CommandResult mission(RobotState & robot_state);

  /**
   * @brief Return to the arming point (flying)
   */
  CommandResult rtl(RobotState & robot_state);

  /// Waypoint being flown (-1 if not in a mission)
  int mission_index() const {return mission_active_ ? static_cast<int>(mission_index_) : -1;}

  /// Number of waypoints of the stored mission
  std::size_t mission_size() const {return mission_.size();}

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
   * current mode, run the automatic transitions and the failsafes
   */
  void update(RobotState & robot_state, double now);

  /**
   * @brief Battery report
   * @param remaining charge 0-1
   * @param now reception time [s]
   */
  void set_battery(double remaining, double now);

  /// Ground station heartbeat received at now [s]
  void set_heartbeat(double now);

  /// Names of the active failsafes ("battery_low", "geofence"...), empty if none
  const std::vector<std::string> & active_failsafes() const {return active_names_;}

  /// Strongest action among the active failsafes
  FailsafeAction failsafe_action() const {return active_action_;}

  /// Pre-arm checks that fail now (empty: can arm)
  std::vector<std::string> prearm_failures(const RobotState & robot_state, double now) const;

  /// Home (arming point) [m]
  const Eigen::Vector3d & home() const {return home_;}

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

  /**
   * @brief Evaluate the failsafes and apply the strongest action
   */
  void update_failsafes(RobotState & robot_state, double dt, double now);

  /// Point inside the geofence closest to p (p itself if inside or no geofence)
  Eigen::Vector3d clamp_to_geofence(const Eigen::Vector3d & p) const;

  /// A failsafe that holds or lands the vehicle is active: reason, empty if none
  std::string blocking_failsafe() const;

  /// Descend at landing_speed keeping XY
  void start_landing(RobotState & robot_state, const std::string & reason);

  /// Go to RTL: climb to the return height first if needed. chained: start from the end
  /// of the current leg (the last waypoint of a mission) instead of the estimate
  void start_rtl(RobotState & robot_state, const std::string & reason, bool chained = false);

  /// Start flying a leg to goal (clamped to the geofence) from the estimate, or from the
  /// end of the previous leg (consecutive waypoints: the setpoint follows the line
  /// between them even if the vehicle went past the waypoint)
  void start_leg(RobotState & robot_state, const Eigen::Vector3d & goal, bool chained = false);

  /// Advance the setpoint along the current leg; true once it is at the goal
  bool advance_leg(RobotState & robot_state, double dt, double speed);

  /// Start waypoint index of the mission
  void start_waypoint(RobotState & robot_state, std::size_t index);

  /// End of the mission or of the RTL: apply action (LOITER, RTL, LAND, DISARM)
  void finish(RobotState & robot_state, FailsafeAction action, const std::string & reason);

  FlightModeParams params_;       ///< Parameters
  ModeCallback mode_callback_;       ///< Mode change notification
  FailsafeCallback failsafe_callback_;       ///< Failsafe change notification

  enum class OffboardType {NONE, POSE, TWIST};
  OffboardType offboard_type_{OffboardType::NONE};       ///< Type of the last offboard setpoint
  Eigen::Vector3d offboard_position_{Eigen::Vector3d::Zero()};       ///< Last pose setpoint
  double offboard_yaw_{0.0};       ///< Yaw of the last pose setpoint [rad]
  Eigen::Vector3d offboard_velocity_{Eigen::Vector3d::Zero()};       ///< Last twist setpoint
  double offboard_yaw_rate_{0.0};       ///< Yaw rate of the last twist setpoint [rad/s]
  double offboard_stamp_{0.0};       ///< Reception time of the last offboard setpoint [s]

  Eigen::Vector3d home_{Eigen::Vector3d::Zero()};       ///< Position when armed [m]
  double takeoff_target_z_{0.0};       ///< Altitude to reach in TAKEOFF [m]
  double landed_time_{0.0};       ///< Time the land detector has been true [s]

  uint64_t last_seq_{0};       ///< Last estimate processed
  double last_stamp_{0.0};       ///< Stamp of that estimate [s]
  bool has_last_{false};       ///< An estimate was processed

  bool has_battery_{false};       ///< A battery was reported
  double battery_remaining_{1.0};       ///< Last charge 0-1
  double battery_stamp_{0.0};       ///< Reception time of the last report [s]
  bool has_heartbeat_{false};       ///< A heartbeat was received
  double heartbeat_stamp_{0.0};       ///< Reception time of the last heartbeat [s]
  double tilt_time_{0.0};       ///< Time beyond max_tilt [s]
  std::vector<Waypoint> mission_;       ///< Stored mission
  std::size_t mission_index_{0};       ///< Waypoint being flown
  bool mission_active_{false};       ///< Flying the mission (MISSION mode)
  bool mission_after_takeoff_{false};       ///< Start the mission when TAKEOFF ends
  double hold_timer_{0.0};       ///< Time at the current waypoint [s]
  Eigen::Vector3d leg_point_{Eigen::Vector3d::Zero()};       ///< Setpoint along the leg
  Eigen::Vector3d leg_goal_{Eigen::Vector3d::Zero()};       ///< End of the leg
  double leg_speed_{0.0};       ///< Current speed of the setpoint along the leg [m/s]
  double leg_acceleration_{0.0};       ///< Acceleration feed-forward along the leg [m/s^2]
  enum class RtlPhase {CLIMB, RETURN};
  RtlPhase rtl_phase_{RtlPhase::CLIMB};       ///< Phase of the return
  std::vector<std::string> active_names_;       ///< Active failsafes
  FailsafeAction active_action_{FailsafeAction::NONE};       ///< Strongest active action
};

/**
 * @brief Yaw of a quaternion
 */
double yaw_from_quaternion(const Eigen::Quaterniond & q);

}  // namespace pilotaro

#endif  // PILOTARO_STATES__FLIGHTMODEMACHINE_HPP_
