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

#ifndef PILOTARO_COMMON__ROBOTSTATE_HPP_
#define PILOTARO_COMMON__ROBOTSTATE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "pilotaro_common/Types.hpp"

namespace pilotaro
{

/**
 * @class RobotState
 * @brief Shared data container used to communicate the pilotaro nodes.
 *
 * PilotaroNode owns a single instance and passes it to the estimator, states,
 * controllers and actuators nodes on each init and cycle call. Every cycle runs in the
 * same thread, so no locking is needed. Each field has a single writer:
 *  - vehicle: PilotaroNode (on configure)
 *  - estimate: EstimatorNode
 *  - mode, setpoint, controller reset: StatesNode
 *  - rotor speeds: ControllersNode
 */
class RobotState
{
public:
  /**
   * @brief Default constructor.
   */
  RobotState() = default;

  /**
   * @brief Virtual destructor.
   */
  virtual ~RobotState() = default;

  /**
   * @brief Gets the robot name (unique in a multirobot system).
   * @return The robot's name as a string.
   */
  const std::string & get_name() const {return name_;}

  /**
   * @brief Sets the robot name.
   * @param new_name The new name to assign.
   */
  void set_name(const std::string & new_name) {name_ = new_name;}

  /**
   * @brief Checks if the robot is available.
   * @return true if available, false otherwise.
   */
  bool is_available() const {return available_;}

  /**
   * @brief Sets the robot availability status.
   * @param new_state true to mark as available, false otherwise.
   */
  void set_available(bool new_state) {available_ = new_state;}

  /**
   * @brief Checks if the robot is in simulation mode.
   * @return true if in simulation, false otherwise.
   */
  bool is_simulation() const {return simulation_;}

  /**
   * @brief Sets the simulation mode status.
   * @param new_state true to enable simulation mode.
   */
  void set_simulation(bool new_state) {simulation_ = new_state;}

  /**
   * @brief Physical properties of the vehicle
   */
  const VehicleParams & get_vehicle() const {return vehicle_;}

  /**
   * @brief Set the physical properties of the vehicle
   * @param vehicle validated parameters
   */
  void set_vehicle(const VehicleParams & vehicle) {vehicle_ = vehicle;}

  /**
   * @brief Last state estimate
   */
  const VehicleState & get_estimate() const {return estimate_;}

  /**
   * @brief Store a new state estimate
   * @param estimate new estimate
   * @param reception_time time it was produced, clock of the nodes [s]
   */
  void set_estimate(const VehicleState & estimate, double reception_time);

  /**
   * @brief Whether at least one estimate has been received
   */
  bool has_estimate() const {return estimate_seq_ > 0;}

  /**
   * @brief Time the last estimate was produced, clock of the nodes [s]
   */
  double get_estimate_reception() const {return estimate_reception_;}

  /**
   * @brief Counter of estimates, increased on each new one (0 = none yet)
   */
  uint64_t get_estimate_seq() const {return estimate_seq_;}

  /**
   * @brief Current flight mode
   */
  FlightMode get_mode() const {return mode_;}

  /**
   * @brief Set the flight mode
   * @param mode new flight mode
   */
  void set_mode(FlightMode mode) {mode_ = mode;}

  /**
   * @brief Reference for the controller
   */
  const ControlSetpoint & get_setpoint() const {return setpoint_;}

  /**
   * @brief Reference for the controller, to be modified in place
   */
  ControlSetpoint & get_setpoint() {return setpoint_;}

  /**
   * @brief Ask the controller to reset its internal state (integrators) on the next cycle
   */
  void request_controller_reset() {controller_reset_ = true;}

  /**
   * @brief Read and clear the controller reset request
   * @return true if a reset was requested
   */
  bool consume_controller_reset();

  /**
   * @brief Rotor speeds computed by the controller [rad/s]
   */
  const std::vector<double> & get_rotor_speeds() const {return rotor_speeds_;}

  /**
   * @brief Store the rotor speeds computed by the controller
   * @param speeds rotor angular velocities [rad/s]
   */
  void set_rotor_speeds(const std::vector<double> & speeds) {rotor_speeds_ = speeds;}

  /**
   * @brief Displays complete robot state information.
   */
  void show() const;

private:
  std::string name_{"pilotaro"};       ///< pilotaro robot name

  bool available_{false};       ///< Indicates if robot is operational

  bool simulation_{false};       ///< pilotaro robot simulation flag

  VehicleParams vehicle_;       ///< Physical properties

  VehicleState estimate_;       ///< Last state estimate

  double estimate_reception_{0.0};       ///< Time the last estimate was produced [s]

  uint64_t estimate_seq_{0};       ///< Number of estimates received

  FlightMode mode_{FlightMode::DISARMED};       ///< Current flight mode

  ControlSetpoint setpoint_;       ///< Reference for the controller

  bool controller_reset_{false};       ///< Pending controller reset

  std::vector<double> rotor_speeds_;       ///< Controller output [rad/s]
};

}  // namespace pilotaro

#endif  // PILOTARO_COMMON__ROBOTSTATE_HPP_
