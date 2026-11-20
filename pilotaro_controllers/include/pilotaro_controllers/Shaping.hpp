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

#ifndef PILOTARO_CONTROLLERS__SHAPING_HPP_
#define PILOTARO_CONTROLLERS__SHAPING_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

#include "pilotaro_common/Types.hpp"

namespace pilotaro_controllers
{

/**
 * @class LowPass3
 * @brief First-order low-pass filter of a 3D vector, for the measured velocity and angular
 * velocity: estimation noise must not reach the actuators.
 */
class LowPass3
{
public:
  /// Cutoff frequency [Hz]; 0 or less: no filtering
  explicit LowPass3(double cutoff = 0.0)
  : cutoff_(cutoff) {}

  /// Filtered value; the first sample (or the first after reset()) passes as it is
  const Eigen::Vector3d & update(const Eigen::Vector3d & x, double dt)
  {
    if (!initialized_ || cutoff_ <= 0.0) {
      value_ = x;
      initialized_ = true;
      return value_;
    }
    const double alpha = dt / (dt + 1.0 / (2.0 * M_PI * cutoff_));
    value_ += alpha * (x - value_);
    return value_;
  }

  void reset() {initialized_ = false;}

private:
  double cutoff_;
  bool initialized_{false};
  Eigen::Vector3d value_{Eigen::Vector3d::Zero()};
};

/**
 * @brief Velocity setpoint of the position loop: feedforward + gain * position error,
 * limited to the max horizontal and vertical speeds.
 *
 * With a position setpoint on the 3 axes the velocity points straight at the goal (gain
 * along the direction of the error, from the per-axis gains) and the whole vector is scaled
 * down when a limit is exceeded, so the vehicle moves along the straight line to the goal.
 * Within kPerAxisDistance of the goal the gain becomes per axis again.
 * Otherwise (velocity control on some axis) each axis is handled on its own. In both cases
 * the speed is capped by setpoint.max_speed, if set.
 */
/// Distance to the goal under which the position gains become per axis [m]
constexpr double kPerAxisDistance = 1.0;

inline Eigen::Vector3d position_to_velocity(
  const pilotaro::ControlSetpoint & setpoint, const Eigen::Vector3d & position,
  const Eigen::Vector3d & gain, double max_horizontal, double max_vertical)
{
  Eigen::Vector3d velocity = setpoint.velocity;
  const Eigen::Vector3d error = setpoint.position - position;
  if (setpoint.xy_position && setpoint.z_position) {
    const double distance = error.norm();
    if (distance > 1e-9) {
      // Far: one gain along the direction of the error (straight line). Within
      // kPerAxisDistance of the goal it blends into the gain of each axis, so every axis
      // converges at its own rate (the path deviation there is a few centimeters)
      const Eigen::Vector3d direction = error / distance;
      const double k = direction.dot(gain.cwiseProduct(direction));
      const double w = std::clamp(1.0 - distance / kPerAxisDistance, 0.0, 1.0);
      velocity += ((1.0 - w) * Eigen::Vector3d::Constant(k) + w * gain).cwiseProduct(error);
    }
    double scale = 1.0;
    const double horizontal = velocity.head<2>().norm();
    if (horizontal > max_horizontal) {
      scale = std::min(scale, max_horizontal / horizontal);
    }
    if (std::abs(velocity.z()) > max_vertical) {
      scale = std::min(scale, max_vertical / std::abs(velocity.z()));
    }
    if (setpoint.max_speed > 0.0 && velocity.norm() > setpoint.max_speed) {
      scale = std::min(scale, setpoint.max_speed / velocity.norm());
    }
    return velocity * scale;
  }
  if (setpoint.xy_position) {
    velocity.head<2>() += gain.head<2>().cwiseProduct(error.head<2>());
  }
  if (setpoint.z_position) {
    velocity.z() += gain.z() * error.z();
  }
  const double horizontal = velocity.head<2>().norm();
  if (horizontal > max_horizontal) {
    velocity.head<2>() *= max_horizontal / horizontal;
  }
  velocity.z() = std::clamp(velocity.z(), -max_vertical, max_vertical);
  if (setpoint.max_speed > 0.0 && velocity.norm() > setpoint.max_speed) {
    velocity *= setpoint.max_speed / velocity.norm();
  }
  return velocity;
}

}  // namespace pilotaro_controllers

#endif  // PILOTARO_CONTROLLERS__SHAPING_HPP_
