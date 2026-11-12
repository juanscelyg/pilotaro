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

#ifndef PILOTARO_ESTIMATORS__ODOMETRYESTIMATOR_HPP_
#define PILOTARO_ESTIMATORS__ODOMETRYESTIMATOR_HPP_

#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pilotaro_core/StateEstimatorBase.hpp"

namespace pilotaro_estimators
{

/**
 * @class OdometryEstimator
 * @brief Base estimator: takes the state directly from a nav_msgs/Odometry topic, as the
 * ACKS autopilot did (ground truth odometry in simulation, or the output of an external
 * estimator such as a VIO or an EKF).
 *
 * Parameters (prefix "estimator."):
 *  - topic (string, "odometry"): odometry topic, relative to the robot namespace
 *  - twist_in_body_frame (bool, true): the twist is in the child frame (REP 147). If
 *    false, the linear velocity is taken as world frame and the angular velocity is
 *    rotated to body frame
 *  - frame_id (string, ""): if set, messages with another header.frame_id are ignored
 *  - use_message_stamp (bool, true): stamp the estimate with the message header (false:
 *    reception time)
 */
class OdometryEstimator : public pilotaro::StateEstimatorBase
{
public:
  bool update(pilotaro::VehicleState & estimate) override;

  void reset() override;

  /**
   * @brief Convert an odometry message to a vehicle state
   * @param msg odometry with pose in world frame
   * @param twist_in_body_frame whether the twist is in the child (body) frame
   * @param[out] state converted state, stamp from the header
   */
  static void to_vehicle_state(
    const nav_msgs::msg::Odometry & msg, bool twist_in_body_frame,
    pilotaro::VehicleState & state);

protected:
  bool on_initialize() override;

private:
  /**
   * @brief Store the last message
   */
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::mutex mutex_;       ///< Protects last_msg_, fresh_ and reception_
  nav_msgs::msg::Odometry::ConstSharedPtr last_msg_;       ///< Last accepted message
  bool fresh_{false};       ///< last_msg_ not consumed yet
  double reception_{0.0};       ///< Reception time of last_msg_ [s]

  bool twist_in_body_frame_{true};       ///< Frame of the twist
  std::string frame_id_;       ///< Expected header.frame_id, empty accepts any
  bool use_message_stamp_{true};       ///< Stamp source

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
};

}  // namespace pilotaro_estimators

#endif  // PILOTARO_ESTIMATORS__ODOMETRYESTIMATOR_HPP_
