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

#ifndef PILOTARO_ESTIMATORS__INSEKFESTIMATOR_HPP_
#define PILOTARO_ESTIMATORS__INSEKFESTIMATOR_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "pilotaro_core/StateEstimatorBase.hpp"
#include "pilotaro_estimators/InsEkf.hpp"

namespace pilotaro_estimators
{

/**
 * @struct InsMeasurement
 * @brief One sensor sample queued for the filter.
 */
struct InsMeasurement
{
  enum class Type {kImu, kGnss, kGnssVelocity, kBaro, kMag, kOdometry};
  Type type{Type::kImu};
  double stamp{0.0};            ///< [s]
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};   ///< accel, ENU position, velocity or field
  Eigen::Vector3d b{Eigen::Vector3d::Zero()};   ///< gyro or odometry velocity (world)
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};   ///< odometry orientation
  double scalar{0.0};           ///< barometric height [m]
};

/**
 * @brief Geodetic (deg, deg, m) to local ENU about an origin (local tangent plane, WGS84
 * radii at the origin; error below 1 mm within a few hundred meters)
 */
Eigen::Vector3d geodetic_to_enu(
  double latitude, double longitude, double altitude, const Eigen::Vector3d & origin);

/**
 * @brief Barometric height of the standard atmosphere [m]
 */
double pressure_to_height(double pressure, double sea_level_pressure);

/**
 * @brief Yaw [rad] from a body-frame magnetic field, levelled with the roll and pitch of
 * orientation; heading: angle of the horizontal world field from world x
 */
double magnetic_yaw(
  const Eigen::Vector3d & field_body, const Eigen::Quaterniond & orientation, double heading);

/**
 * @class InsEkfEstimator
 * @brief Inertial navigation estimator: IMU-driven error-state EKF (InsEkf) corrected with
 * GNSS, barometer, magnetometer and, optionally, an external odometry (VIO, DVL + USBL,
 * motion capture...).
 *
 * It starts once the vehicle has been still for init_time: roll and pitch from the
 * accelerometer, gyro bias from the mean gyro, yaw from the magnetometer (or the odometry),
 * position from the first GNSS fix (or the odometry). Then it gives an estimate on each
 * IMU sample. Sensor samples are fused in time order.
 *
 * Parameters (prefix "estimator."), an empty topic disables that sensor:
 *  - imu_topic ("imu"), gnss_topic ("navsat"), gnss_velocity_topic (""), baro_topic
 *    ("air_pressure"), mag_topic ("magnetometer"), odometry_topic ("")
 *  - gnss_origin ([lat, lon, alt], empty: the first fix is the origin)
 *  - gnss_noise ([horizontal, vertical] m), gnss_velocity_noise (m/s), baro_noise (m),
 *    sea_level_pressure (Pa), mag_heading (rad, direction of the horizontal world field
 *    from world x), mag_noise (per axis of the unit field vector; the field direction is
 *    fused in 3D, its inclination taken at the start)
 *  - odometry_position_noise (m), odometry_velocity_noise (m/s, 0 = not fused),
 *    odometry_attitude_noise (rad per axis, 0 = not fused), odometry_yaw_noise (rad, only
 *    the yaw, 0 = not fused)
 *  - gyro_noise, accel_noise, gyro_bias_noise, accel_bias_noise, baro_bias_noise (process
 *    noise densities), innovation_gate (sigmas, 0 = none), init_time (s)
 *  - reset_timeout (s, 0 = never): a sensor rejected by the gate for longer than this
 *    resets its state (position, velocity or barometer offset) to the measurement, as
 *    EKF2 does. It recovers the filter after an impact the IMU did not sample (touchdown)
 *  - aiding_timeout (s, 0 = never): without an accepted position (GNSS or odometry) for
 *    longer than this, the estimate reports an unknown position (position_std 1 km), as
 *    EKF2_NOAID_TOUT, so the navigation failsafe of the states node can act
 */
class InsEkfEstimator : public pilotaro::StateEstimatorBase
{
public:
  bool update(pilotaro::VehicleState & estimate) override;

  void reset() override;

  /// Filter in use (tests, diagnostics)
  const InsEkf & filter() const {return filter_;}

  /// Number of state resets after sustained rejections
  int resets() const {return resets_;}

  /// Process one sample now (tests); returns true if it was an IMU sample with an estimate
  bool process(const InsMeasurement & m);

  /// Estimate after the last IMU sample processed, with its position uncertainty
  void current_estimate(pilotaro::VehicleState & estimate);

protected:
  bool on_initialize() override;

private:
  void push(const InsMeasurement & m);

  /// The odometry gives the heading (yaw or whole attitude fused)
  bool odometry_heading() const
  {
    return use_odometry_ && (odometry_yaw_sigma_ > 0.0 || odometry_attitude_sigma_ > 0.0);
  }
  bool try_initialize(const InsMeasurement & m);

  // Parameters
  std::optional<Eigen::Vector3d> origin_;
  Eigen::Vector3d gnss_sigma_{0.5, 0.5, 1.0};
  double gnss_velocity_sigma_{0.1};
  double baro_sigma_{0.5};
  double sea_level_pressure_{101325.0};
  double mag_heading_{0.0};
  double mag_sigma_{0.05};
  Eigen::Vector3d mag_world_{Eigen::Vector3d::UnitX()};   ///< Unit world field
  double odometry_position_sigma_{0.05};
  double odometry_velocity_sigma_{0.0};
  double odometry_yaw_sigma_{0.0};
  double odometry_attitude_sigma_{0.0};
  double gate_{5.0};
  double init_time_{1.0};
  double reset_timeout_{1.0};
  double aiding_timeout_{5.0};
  bool aiding_lost_{false};
  bool use_gnss_{true};
  bool use_baro_{true};
  bool use_mag_{true};
  bool use_odometry_{false};

  // Queue filled by the callbacks
  std::mutex mutex_;
  std::vector<InsMeasurement> queue_;
  std::vector<InsMeasurement> batch_;     ///< Samples being processed by update()

  // Filter and initialization
  InsEkf filter_;
  double last_imu_stamp_{-1.0};
  double still_since_{-1.0};
  Eigen::Vector3d accel_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_sum_{Eigen::Vector3d::Zero()};
  int still_samples_{0};
  std::optional<Eigen::Vector3d> last_position_;
  std::optional<Eigen::Vector3d> last_field_;
  std::optional<double> last_baro_;
  std::optional<Eigen::Quaterniond> last_odometry_orientation_;
  Eigen::Vector3d last_gyro_{Eigen::Vector3d::Zero()};

  /// Time each aiding measurement was last accepted (or reset), indexed by Type
  double accepted_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /// Fuse with update(); if rejected for longer than reset_timeout, reset(). Returns true
  /// if the measurement changed the state
  template<typename Update, typename Reset>
  bool fuse_or_reset(InsMeasurement::Type type, double stamp, Update update, Reset reset);

  int resets_{0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    gnss_velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
};

}  // namespace pilotaro_estimators

#endif  // PILOTARO_ESTIMATORS__INSEKFESTIMATOR_HPP_
