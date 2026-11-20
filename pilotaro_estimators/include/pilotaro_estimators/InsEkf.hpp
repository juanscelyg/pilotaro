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

#ifndef PILOTARO_ESTIMATORS__INSEKF_HPP_
#define PILOTARO_ESTIMATORS__INSEKF_HPP_

#include <Eigen/Dense>

namespace pilotaro_estimators
{

/**
 * @struct InsEkfParams
 * @brief Process noise of the inertial navigation filter (continuous-time densities).
 */
struct InsEkfParams
{
  double gyro_noise{1.0e-3};          ///< Gyro white noise [rad/s/sqrt(Hz)]
  double accel_noise{4.0e-3};         ///< Accelerometer white noise [m/s^2/sqrt(Hz)]
  double gyro_bias_noise{1.0e-5};     ///< Gyro bias random walk [rad/s^2/sqrt(Hz)]
  double accel_bias_noise{1.0e-4};    ///< Accelerometer bias random walk [m/s^3/sqrt(Hz)]
  double baro_bias_noise{5.0e-3};     ///< Barometer offset random walk [m/s/sqrt(Hz)]
  double gravity{9.8};                ///< [m/s^2], along -z of the world (ENU)
};

/**
 * @struct InsEkfSigmas
 * @brief Initial standard deviations of the error state.
 */
struct InsEkfSigmas
{
  double position{0.5};       ///< [m]
  double velocity{0.1};       ///< [m/s]
  double tilt{0.05};          ///< Roll and pitch [rad]
  double yaw{0.1};            ///< [rad]
  double accel_bias{0.1};     ///< [m/s^2]
  double gyro_bias{0.005};    ///< [rad/s]
  double baro_bias{0.5};      ///< [m], anti-correlated with the height (see initialize())
};

/**
 * @class InsEkf
 * @brief Error-state extended Kalman filter for inertial navigation (no ROS).
 *
 * Nominal state: position and velocity in the world frame (ENU), orientation body (FLU) ->
 * world, accelerometer and gyro biases (body) and the offset of the barometric height.
 * Error state (16): dp, dv, dtheta (local, R = R_nominal * Exp(dtheta)), dba, dbg, dbb.
 *
 * predict() integrates the IMU (specific force and angular velocity, bias corrected); the
 * update_*() functions fuse a measurement, rejecting it if its normalized innovation is
 * beyond the gate (gate^2 per dimension). After each update the error is injected into the
 * nominal state and reset.
 */
class InsEkf
{
public:
  static constexpr int kSize = 16;
  using Vector = Eigen::Matrix<double, kSize, 1>;
  using Matrix = Eigen::Matrix<double, kSize, kSize>;

  // Offsets of the error state blocks
  static constexpr int kP = 0;
  static constexpr int kV = 3;
  static constexpr int kTheta = 6;
  static constexpr int kAccelBias = 9;
  static constexpr int kGyroBias = 12;
  static constexpr int kBaroBias = 15;

  explicit InsEkf(const InsEkfParams & params = InsEkfParams())
  : params_(params) {}

  /**
   * @brief Start the filter
   * @param position world position [m]
   * @param orientation body -> world
   * @param gyro_bias initial gyro bias (e.g. mean of the gyro at rest) [rad/s]
   * @param baro_height barometric height at that moment [m]; the offset starts as
   * baro_height - position.z, anti-correlated with the height
   * @param sigmas initial uncertainty
   */
  void initialize(
    const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & gyro_bias, double baro_height, const InsEkfSigmas & sigmas);

  bool initialized() const {return initialized_;}

  /// Forget the state; initialize() is needed again
  void reset() {initialized_ = false;}

  /**
   * @brief Propagate with an IMU sample
   * @param accel specific force in body frame [m/s^2]
   * @param gyro angular velocity in body frame [rad/s]
   * @param dt time since the previous sample [s]
   */
  void predict(const Eigen::Vector3d & accel, const Eigen::Vector3d & gyro, double dt);

  /// World position measurement with per-axis standard deviation; false if rejected
  bool update_position(
    const Eigen::Vector3d & position, const Eigen::Vector3d & sigma, double gate);

  /// World velocity measurement with per-axis standard deviation; false if rejected
  bool update_velocity(
    const Eigen::Vector3d & velocity, const Eigen::Vector3d & sigma, double gate);

  /// Barometric height (= z + offset); false if rejected
  bool update_baro(double height, double sigma, double gate);

  /// Yaw measurement (heading of body x from world x, about world z); false if rejected
  bool update_yaw(double yaw, double sigma, double gate);

  /// Orientation measurement (body -> world), sigma per axis [rad]; false if rejected
  bool update_orientation(const Eigen::Quaterniond & orientation, double sigma, double gate);

  /**
   * @brief Direction of a known world vector seen in the body frame (magnetometer):
   * body = R^T * world, both unit vectors; sigma per axis. Couples heading and tilt
   * correctly, unlike a heading computed with the estimated tilt. False if rejected
   */
  bool update_direction(
    const Eigen::Vector3d & body, const Eigen::Vector3d & world, double sigma, double gate);

  /**
   * @brief Set the position to a measurement, forgetting its correlations (recovery when a
   * sensor keeps being rejected, e.g. after an impact the IMU did not sample)
   */
  void reset_position(const Eigen::Vector3d & position, const Eigen::Vector3d & sigma);

  /// Set the velocity to a measurement, forgetting its correlations
  void reset_velocity(const Eigen::Vector3d & velocity, const Eigen::Vector3d & sigma);

  /// Set the barometer offset so that the height matches the current z
  void reset_baro_bias(double height, double sigma);

  /**
   * @brief Set z to the barometric height (keeping the offset) and open the vertical
   * velocity: the barometer is the height reference when there is no GNSS
   */
  void reset_height(double baro_height, double sigma);

  const Eigen::Vector3d & position() const {return p_;}
  const Eigen::Vector3d & velocity() const {return v_;}
  const Eigen::Quaterniond & orientation() const {return q_;}
  const Eigen::Vector3d & accel_bias() const {return ba_;}
  const Eigen::Vector3d & gyro_bias() const {return bg_;}
  double baro_bias() const {return bb_;}
  const Matrix & covariance() const {return P_;}

  /// Yaw of the current orientation [rad]
  double yaw() const;

  /// Normalized innovation squared of the last update (accepted or not)
  double last_nis() const {return last_nis_;}

private:
  template<int M>
  bool correct(
    const Eigen::Matrix<double, M, 1> & residual, const Eigen::Matrix<double, M, kSize> & H,
    const Eigen::Matrix<double, M, 1> & variance, double gate);

  void inject(const Vector & dx);

  /// Zero the correlations of state i and set its variance
  void decorrelate(int i, double variance);

  InsEkfParams params_;
  bool initialized_{false};
  Eigen::Vector3d p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
  double bb_{0.0};
  Matrix P_{Matrix::Identity()};
  double last_nis_{0.0};
};

}  // namespace pilotaro_estimators

#endif  // PILOTARO_ESTIMATORS__INSEKF_HPP_
