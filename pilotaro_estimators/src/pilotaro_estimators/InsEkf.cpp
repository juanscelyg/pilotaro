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

#include <algorithm>
#include <cmath>

#include "pilotaro_estimators/InsEkf.hpp"

namespace pilotaro_estimators
{

namespace
{
Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Quaterniond exp_quaternion(const Eigen::Vector3d & theta)
{
  const double angle = theta.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond(1.0, 0.5 * theta.x(), 0.5 * theta.y(), 0.5 * theta.z())
           .normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, theta / angle));
}

// Velocity variance after a position reset [(m/s)^2]
constexpr double kResetVelocityVariance = 0.25;

double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

void
InsEkf::initialize(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation,
  const Eigen::Vector3d & gyro_bias, double baro_height, const InsEkfSigmas & s)
{
  p_ = position;
  v_.setZero();
  q_ = orientation.normalized();
  ba_.setZero();
  bg_ = gyro_bias;
  bb_ = baro_height - position.z();

  P_.setZero();
  P_.block<3, 3>(kP, kP).diagonal().setConstant(s.position * s.position);
  P_.block<3, 3>(kV, kV).diagonal().setConstant(s.velocity * s.velocity);
  // Attitude: roll and pitch about the local axes, yaw about the world z
  const Eigen::Matrix3d R = q_.toRotationMatrix();
  const Eigen::Vector3d z_body = R.transpose() * Eigen::Vector3d::UnitZ();
  P_.block<3, 3>(kTheta, kTheta) =
    s.tilt * s.tilt * (Eigen::Matrix3d::Identity() - z_body * z_body.transpose()) +
    s.yaw * s.yaw * z_body * z_body.transpose();
  P_.block<3, 3>(kAccelBias, kAccelBias).diagonal().setConstant(s.accel_bias * s.accel_bias);
  P_.block<3, 3>(kGyroBias, kGyroBias).diagonal().setConstant(s.gyro_bias * s.gyro_bias);
  // The offset comes from the same height: their errors are opposite
  P_(kBaroBias, kBaroBias) = s.baro_bias * s.baro_bias + s.position * s.position;
  P_(kBaroBias, kP + 2) = -s.position * s.position;
  P_(kP + 2, kBaroBias) = -s.position * s.position;
  initialized_ = true;
}

void
InsEkf::predict(const Eigen::Vector3d & accel, const Eigen::Vector3d & gyro, double dt)
{
  if (!initialized_ || !(dt > 0.0)) {
    return;
  }
  const Eigen::Vector3d a = accel - ba_;
  const Eigen::Vector3d w = gyro - bg_;
  const Eigen::Matrix3d R = q_.toRotationMatrix();
  const Eigen::Vector3d acceleration = R * a - params_.gravity * Eigen::Vector3d::UnitZ();

  p_ += v_ * dt + 0.5 * acceleration * dt * dt;
  v_ += acceleration * dt;
  q_ = (q_ * exp_quaternion(w * dt)).normalized();

  Matrix F = Matrix::Identity();
  F.block<3, 3>(kP, kV) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(kV, kTheta) = -R * skew(a) * dt;
  F.block<3, 3>(kV, kAccelBias) = -R * dt;
  F.block<3, 3>(kTheta, kTheta) = exp_quaternion(w * dt).toRotationMatrix().transpose();
  F.block<3, 3>(kTheta, kGyroBias) = -Eigen::Matrix3d::Identity() * dt;

  Vector q = Vector::Zero();
  q.segment<3>(kV).setConstant(params_.accel_noise * params_.accel_noise * dt);
  q.segment<3>(kTheta).setConstant(params_.gyro_noise * params_.gyro_noise * dt);
  q.segment<3>(kAccelBias).setConstant(
    params_.accel_bias_noise * params_.accel_bias_noise * dt);
  q.segment<3>(kGyroBias).setConstant(params_.gyro_bias_noise * params_.gyro_bias_noise * dt);
  q(kBaroBias) = params_.baro_bias_noise * params_.baro_bias_noise * dt;

  P_ = F * P_ * F.transpose();
  P_.diagonal() += q;
  P_ = 0.5 * (P_ + P_.transpose());
}

template<int M>
bool
InsEkf::correct(
  const Eigen::Matrix<double, M, 1> & residual, const Eigen::Matrix<double, M, kSize> & H,
  const Eigen::Matrix<double, M, 1> & variance, double gate)
{
  if (!initialized_) {
    return false;
  }
  const Eigen::Matrix<double, M, M> R = variance.asDiagonal();
  const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  const Eigen::LDLT<Eigen::Matrix<double, M, M>> S_ldlt(S);
  last_nis_ = residual.dot(S_ldlt.solve(residual));
  if (gate > 0.0 && last_nis_ > gate * gate * M) {
    return false;
  }
  const Eigen::Matrix<double, kSize, M> K =
    S_ldlt.solve(H * P_.transpose()).transpose();
  const Matrix I_KH = Matrix::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  inject(K * residual);
  return true;
}

void
InsEkf::inject(const Vector & dx)
{
  p_ += dx.segment<3>(kP);
  v_ += dx.segment<3>(kV);
  q_ = (q_ * exp_quaternion(dx.segment<3>(kTheta))).normalized();
  ba_ += dx.segment<3>(kAccelBias);
  bg_ += dx.segment<3>(kGyroBias);
  bb_ += dx(kBaroBias);
}

bool
InsEkf::update_position(
  const Eigen::Vector3d & position, const Eigen::Vector3d & sigma, double gate)
{
  Eigen::Matrix<double, 3, kSize> H = Eigen::Matrix<double, 3, kSize>::Zero();
  H.block<3, 3>(0, kP).setIdentity();
  return correct<3>(position - p_, H, sigma.cwiseProduct(sigma), gate);
}

bool
InsEkf::update_velocity(
  const Eigen::Vector3d & velocity, const Eigen::Vector3d & sigma, double gate)
{
  Eigen::Matrix<double, 3, kSize> H = Eigen::Matrix<double, 3, kSize>::Zero();
  H.block<3, 3>(0, kV).setIdentity();
  return correct<3>(velocity - v_, H, sigma.cwiseProduct(sigma), gate);
}

bool
InsEkf::update_baro(double height, double sigma, double gate)
{
  Eigen::Matrix<double, 1, kSize> H = Eigen::Matrix<double, 1, kSize>::Zero();
  H(0, kP + 2) = 1.0;
  H(0, kBaroBias) = 1.0;
  return correct<1>(
    Eigen::Matrix<double, 1, 1>(height - (p_.z() + bb_)), H,
    Eigen::Matrix<double, 1, 1>(sigma * sigma), gate);
}

void
InsEkf::decorrelate(int i, double variance)
{
  P_.row(i).setZero();
  P_.col(i).setZero();
  P_(i, i) = variance;
}

void
InsEkf::reset_position(const Eigen::Vector3d & position, const Eigen::Vector3d & sigma)
{
  if (!initialized_) {
    return;
  }
  // The barometer offset keeps the barometric height consistent with the new z
  bb_ += p_.z() - position.z();
  p_ = position;
  for (int i = 0; i < 3; ++i) {
    decorrelate(kP + i, sigma[i] * sigma[i]);
    // The velocity that took the position away is suspect too: let the next
    // measurements correct it
    decorrelate(kV + i, std::max(P_(kV + i, kV + i), kResetVelocityVariance));
  }
}

void
InsEkf::reset_velocity(const Eigen::Vector3d & velocity, const Eigen::Vector3d & sigma)
{
  if (!initialized_) {
    return;
  }
  v_ = velocity;
  for (int i = 0; i < 3; ++i) {
    decorrelate(kV + i, sigma[i] * sigma[i]);
  }
}

void
InsEkf::reset_baro_bias(double height, double sigma)
{
  if (!initialized_) {
    return;
  }
  bb_ = height - p_.z();
  decorrelate(kBaroBias, sigma * sigma + P_(kP + 2, kP + 2));
}

void
InsEkf::reset_height(double baro_height, double sigma)
{
  if (!initialized_) {
    return;
  }
  p_.z() = baro_height - bb_;
  decorrelate(kP + 2, sigma * sigma);
  decorrelate(kV + 2, std::max(P_(kV + 2, kV + 2), kResetVelocityVariance));
}

double
InsEkf::yaw() const
{
  return std::atan2(
    2.0 * (q_.w() * q_.z() + q_.x() * q_.y()), 1.0 - 2.0 * (q_.y() * q_.y() + q_.z() * q_.z()));
}

bool
InsEkf::update_yaw(double yaw_measured, double sigma, double gate)
{
  // A local rotation dtheta turns the vehicle by R * dtheta in the world: yaw += z of it
  Eigen::Matrix<double, 1, kSize> H = Eigen::Matrix<double, 1, kSize>::Zero();
  H.block<1, 3>(0, kTheta) = q_.toRotationMatrix().row(2);
  return correct<1>(
    Eigen::Matrix<double, 1, 1>(wrap_angle(yaw_measured - yaw())), H,
    Eigen::Matrix<double, 1, 1>(sigma * sigma), gate);
}

bool
InsEkf::update_orientation(const Eigen::Quaterniond & orientation, double sigma, double gate)
{
  // R_meas = R * Exp(dtheta): the residual is the local rotation from the estimate
  const Eigen::AngleAxisd d(q_.conjugate() * orientation.normalized());
  const Eigen::Vector3d residual = std::remainder(d.angle(), 2.0 * M_PI) * d.axis();
  Eigen::Matrix<double, 3, kSize> H = Eigen::Matrix<double, 3, kSize>::Zero();
  H.block<3, 3>(0, kTheta).setIdentity();
  return correct<3>(residual, H, Eigen::Vector3d::Constant(sigma * sigma), gate);
}

bool
InsEkf::update_direction(
  const Eigen::Vector3d & body, const Eigen::Vector3d & world, double sigma, double gate)
{
  // R_true = R * Exp(dtheta): R_true^T w ~= R^T w + [R^T w]x dtheta
  const Eigen::Vector3d predicted = q_.conjugate() * world.normalized();
  Eigen::Matrix<double, 3, kSize> H = Eigen::Matrix<double, 3, kSize>::Zero();
  H.block<3, 3>(0, kTheta) = skew(predicted);
  return correct<3>(
    body.normalized() - predicted, H, Eigen::Vector3d::Constant(sigma * sigma), gate);
}

}  // namespace pilotaro_estimators
