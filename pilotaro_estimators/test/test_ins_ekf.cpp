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

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "pilotaro_estimators/InsEkf.hpp"
#include "pilotaro_estimators/InsEkfEstimator.hpp"

namespace pilotaro_estimators
{

namespace
{

constexpr double kG = 9.8;
constexpr double kDt = 0.004;   // 250 Hz IMU

double yaw_of(const Eigen::Quaterniond & q)
{
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

double angle_between(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  return Eigen::AngleAxisd(a.conjugate() * b).angle();
}

// Simulated flight: Lissajous path with varying yaw, roll and pitch (a constant-rate turn
// would leave tilt, accelerometer and gyro biases unobservable), with IMU biases and the
// noise of the pilotaro simulated sensors
struct Flight
{
  Eigen::Vector3d accel_bias{0.05, -0.03, 0.04};
  Eigen::Vector3d gyro_bias{0.003, -0.002, 0.004};
  double baro_offset{3.0};
  std::mt19937 rng{7};
  std::normal_distribution<double> n{0.0, 1.0};

  Eigen::Vector3d position(double t) const
  {
    return {3.0 * std::sin(0.4 * t), 2.0 * std::sin(0.6 * t), 2.0 + 0.5 * std::sin(0.3 * t)};
  }
  Eigen::Vector3d velocity(double t) const
  {
    return {1.2 * std::cos(0.4 * t), 1.2 * std::cos(0.6 * t), 0.15 * std::cos(0.3 * t)};
  }
  Eigen::Vector3d acceleration(double t) const
  {
    return {-0.48 * std::sin(0.4 * t), -0.72 * std::sin(0.6 * t), -0.045 * std::sin(0.3 * t)};
  }
  Eigen::Quaterniond orientation(double t) const
  {
    return Eigen::Quaterniond(
      Eigen::AngleAxisd(0.8 * std::sin(0.25 * t), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.1 * std::sin(0.5 * t), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.1 * std::cos(0.35 * t), Eigen::Vector3d::UnitX()));
  }
  Eigen::Vector3d body_rate(double t) const
  {
    const double h = 1e-4;
    const Eigen::AngleAxisd d(
      orientation(t - 0.5 * h).conjugate() * orientation(t + 0.5 * h));
    return d.angle() * d.axis() / h;
  }
  Eigen::Vector3d noise(double sigma)
  {
    return sigma * Eigen::Vector3d(n(rng), n(rng), n(rng));
  }
  void imu(double t, Eigen::Vector3d & accel, Eigen::Vector3d & gyro)
  {
    const Eigen::Matrix3d R = orientation(t).toRotationMatrix();
    accel = R.transpose() * (acceleration(t) + kG * Eigen::Vector3d::UnitZ()) + accel_bias +
      noise(0.021);
    gyro = body_rate(t) + gyro_bias + noise(0.009);
  }
};

InsEkfParams sim_params()
{
  InsEkfParams p;
  p.gravity = kG;
  return p;
}

}  // namespace

TEST(InsEkf, FollowsAFlightWithBiasedNoisySensors)
{
  Flight flight;
  InsEkf ekf(sim_params());
  // Start 1 m off, 0.15 rad of yaw error and without knowing the biases
  ekf.initialize(
    flight.position(0.0) + Eigen::Vector3d(1.0, -0.5, 0.3),
    flight.orientation(0.0) * Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d::Zero(), flight.position(0.0).z() + flight.baro_offset, InsEkfSigmas());
  ekf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0);   // dt 0: ignored

  double sum_p = 0.0, sum_v = 0.0, max_att = 0.0;
  int samples = 0;
  const int steps = static_cast<int>(120.0 / kDt);
  for (int k = 1; k <= steps; ++k) {
    const double t = k * kDt;
    Eigen::Vector3d accel, gyro;
    flight.imu(t, accel, gyro);
    ekf.predict(accel, gyro, kDt);
    if (k % 25 == 0) {     // GNSS 10 Hz
      ekf.update_position(
        flight.position(t) + Eigen::Vector3d(flight.noise(0.5).x(), flight.noise(0.5).y(),
        flight.noise(1.0).z()), Eigen::Vector3d(0.5, 0.5, 1.0), 5.0);
    }
    if (k % 5 == 0) {      // baro and magnetometer 50 Hz
      ekf.update_baro(
        flight.position(t).z() + flight.baro_offset + flight.noise(0.1).x(), 0.1, 5.0);
      ekf.update_yaw(yaw_of(flight.orientation(t)) + flight.noise(0.02).x(), 0.05, 5.0);
    }
    if (t > 90.0) {
      sum_p += (ekf.position() - flight.position(t)).squaredNorm();
      sum_v += (ekf.velocity() - flight.velocity(t)).squaredNorm();
      max_att = std::max(max_att, angle_between(ekf.orientation(), flight.orientation(t)));
      ++samples;
    }
  }
  EXPECT_LT(std::sqrt(sum_p / samples), 0.3);     // GNSS alone is 0.5 m / 1 m
  EXPECT_LT(std::sqrt(sum_v / samples), 0.1);
  EXPECT_LT(max_att, 0.02);                       // ~1 degree
  EXPECT_LT((ekf.gyro_bias() - flight.gyro_bias).norm(), 0.001);
  EXPECT_NEAR(ekf.baro_bias(), flight.baro_offset, 0.3);
}

TEST(InsEkf, LearnsTheBiasesAtRest)
{
  InsEkf ekf(sim_params());
  const Eigen::Vector3d gyro_bias(0.004, -0.003, 0.002);
  const double accel_bias_z = 0.08;
  ekf.initialize(
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0,
    InsEkfSigmas());
  for (int k = 1; k <= static_cast<int>(60.0 / kDt); ++k) {
    ekf.predict(Eigen::Vector3d(0.0, 0.0, kG + accel_bias_z), gyro_bias, kDt);
    if (k % 25 == 0) {
      ekf.update_position(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.5, 1.0), 5.0);
    }
    if (k % 5 == 0) {
      ekf.update_baro(0.0, 0.1, 5.0);
      ekf.update_yaw(0.0, 0.05, 5.0);
    }
  }
  EXPECT_LT((ekf.gyro_bias() - gyro_bias).norm(), 2e-4);
  EXPECT_NEAR(ekf.accel_bias().z(), accel_bias_z, 0.01);
  EXPECT_LT(ekf.velocity().norm(), 0.02);
}

TEST(InsEkf, MagneticDirectionCorrectsYawAndTilt)
{
  // Field inclined 63 degrees, as in the gz worlds; start 0.2 rad off in yaw, 0.02 in roll.
  // One direction fixes two angles: gravity (the vertical) gives the third
  const Eigen::Vector3d field = Eigen::Vector3d(0.216, 0.0097, 0.429).normalized();
  const Eigen::Quaterniond truth(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  InsEkf ekf(sim_params());
  ekf.initialize(
    Eigen::Vector3d::Zero(),
    truth * Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitX())),
    Eigen::Vector3d::Zero(), 0.0, InsEkfSigmas());
  for (int k = 0; k < 200; ++k) {
    EXPECT_TRUE(ekf.update_direction(truth.conjugate() * field, field, 0.02, 0.0));
    EXPECT_TRUE(
      ekf.update_direction(
        truth.conjugate() * Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ(), 0.02, 0.0));
  }
  EXPECT_LT(angle_between(ekf.orientation(), truth), 0.005);
  EXPECT_NEAR(yaw_of(ekf.orientation()), 0.7, 0.005);
}

TEST(InsEkf, OrientationMeasurementIsFused)
{
  const Eigen::Quaterniond truth(
    Eigen::AngleAxisd(2.5, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  InsEkf ekf(sim_params());
  ekf.initialize(
    Eigen::Vector3d::Zero(),
    truth * Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d(1, -1, 2).normalized())),
    Eigen::Vector3d::Zero(), 0.0, InsEkfSigmas());
  for (int k = 0; k < 50; ++k) {
    EXPECT_TRUE(ekf.update_orientation(truth, 0.01, 5.0));
  }
  EXPECT_LT(angle_between(ekf.orientation(), truth), 1e-3);
  // Across +-pi of yaw the residual is the short way round
  const Eigen::Quaterniond across(Eigen::AngleAxisd(-3.1, Eigen::Vector3d::UnitZ()));
  ekf.initialize(
    Eigen::Vector3d::Zero(), Eigen::Quaterniond(Eigen::AngleAxisd(3.1, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d::Zero(), 0.0, InsEkfSigmas());
  for (int k = 0; k < 50; ++k) {
    ekf.update_orientation(across, 0.01, 0.0);
  }
  EXPECT_LT(angle_between(ekf.orientation(), across), 1e-3);
}

TEST(InsEkf, HeightResetKeepsTheBarometerOffset)
{
  InsEkf ekf(sim_params());
  ekf.initialize(
    Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
    11.0, InsEkfSigmas());
  ASSERT_NEAR(ekf.baro_bias(), 10.0, 1e-12);
  ekf.reset_height(13.0, 0.3);       // barometer says 3 m
  EXPECT_NEAR(ekf.position().z(), 3.0, 1e-12);
  EXPECT_NEAR(ekf.baro_bias(), 10.0, 1e-12);
  EXPECT_NEAR(ekf.covariance()(InsEkf::kP + 2, InsEkf::kP + 2), 0.09, 1e-12);
  EXPECT_NEAR(ekf.covariance()(InsEkf::kP + 2, InsEkf::kBaroBias), 0.0, 1e-12);
}

TEST(InsEkf, RejectsOutliers)
{
  InsEkf ekf(sim_params());
  ekf.initialize(
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0,
    InsEkfSigmas());
  EXPECT_FALSE(ekf.update_position(Eigen::Vector3d(50.0, 0.0, 0.0), Eigen::Vector3d::Ones(), 5.0));
  EXPECT_NEAR(ekf.position().x(), 0.0, 1e-12);
  EXPECT_GT(ekf.last_nis(), 75.0);
  EXPECT_TRUE(ekf.update_position(Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Vector3d::Ones(), 5.0));
  EXPECT_GT(ekf.position().x(), 0.0);
  // Without gate everything is fused
  EXPECT_TRUE(ekf.update_position(Eigen::Vector3d(50.0, 0.0, 0.0), Eigen::Vector3d::Ones(), 0.0));
}

TEST(InsEkf, NothingHappensBeforeInitialize)
{
  InsEkf ekf;
  ekf.predict(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), kDt);
  EXPECT_FALSE(ekf.update_baro(1.0, 0.1, 5.0));
  EXPECT_FALSE(ekf.initialized());
  EXPECT_DOUBLE_EQ(ekf.position().norm(), 0.0);
}

TEST(InsHelpers, GeodeticToEnu)
{
  const Eigen::Vector3d origin(47.0, 8.0, 100.0);
  const Eigen::Vector3d north = geodetic_to_enu(47.00001, 8.0, 100.0, origin);
  EXPECT_NEAR(north.y(), 1.1117, 0.002);    // 1e-5 deg of latitude
  EXPECT_NEAR(north.x(), 0.0, 1e-9);
  const Eigen::Vector3d east = geodetic_to_enu(47.0, 8.00001, 102.5, origin);
  EXPECT_NEAR(east.x(), 1.1117 * std::cos(47.0 * M_PI / 180.0), 0.003);
  EXPECT_NEAR(east.z(), 2.5, 1e-9);
}

TEST(InsHelpers, PressureToHeight)
{
  EXPECT_NEAR(pressure_to_height(101325.0, 101325.0), 0.0, 1e-9);
  // ~12 Pa per meter near sea level
  EXPECT_NEAR(pressure_to_height(101325.0 - 12.0, 101325.0), 1.0, 0.02);
}

TEST(InsHelpers, MagneticYawIgnoresTilt)
{
  const Eigen::Vector3d field_world(0.216, 0.0097, 0.429);   // as in the gz worlds
  const double heading = std::atan2(field_world.y(), field_world.x());
  for (const double yaw : {-2.5, -0.3, 0.0, 1.0, 3.0}) {
    const Eigen::Quaterniond q(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitX()));
    const Eigen::Vector3d body = q.conjugate() * field_world;
    // Levelled with the true tilt, whatever yaw the orientation has
    const Eigen::Quaterniond wrong_yaw =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) * q;
    EXPECT_NEAR(magnetic_yaw(body, q, heading), yaw, 1e-9);
    EXPECT_NEAR(magnetic_yaw(body, wrong_yaw, heading), yaw, 1e-9);
  }
}

class InsEkfEstimatorTest : public ::testing::Test
{
protected:
  void SetUp() override {start({0.5, 1.0});}

  void start(const std::vector<double> & gnss_noise)
  {
    node_ = std::make_shared<rclcpp::Node>(
      "ins_test", rclcpp::NodeOptions().parameter_overrides(
    {
      {"estimator.gnss_origin", std::vector<double>{47.0, 8.0, 0.0}},
      {"estimator.mag_heading", std::atan2(0.0097, 0.216)},
      {"estimator.init_time", 0.5},
      {"estimator.gnss_noise", gnss_noise},
    }));
    estimator_ = std::make_unique<InsEkfEstimator>();
    pilotaro::VehicleParams vehicle;
    vehicle.gravity = kG;
    ASSERT_TRUE(estimator_->initialize(node_, "estimator", vehicle));
  }

  InsMeasurement imu(double t, const Eigen::Vector3d & accel = Eigen::Vector3d(0, 0, kG))
  {
    InsMeasurement m;
    m.type = InsMeasurement::Type::kImu;
    m.stamp = t;
    m.a = accel;
    m.b = Eigen::Vector3d(0.002, -0.001, 0.003);    // gyro bias at rest
    return m;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<InsEkfEstimator> estimator_;
};

TEST_F(InsEkfEstimatorTest, StartsWhenStillAndAllSensorsArrived)
{
  const double yaw = 0.5;
  InsMeasurement mag;
  mag.type = InsMeasurement::Type::kMag;
  mag.a = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::Vector3d(0.216, 0.0097, 0.429);
  InsMeasurement gnss;
  gnss.type = InsMeasurement::Type::kGnss;
  gnss.a = Eigen::Vector3d(1.0, 2.0, 0.3);
  InsMeasurement baro;
  baro.type = InsMeasurement::Type::kBaro;
  baro.scalar = 5.3;

  // A shaken IMU restarts the still period
  double t = 0.0;
  for (; t < 0.4; t += kDt) {
    EXPECT_FALSE(estimator_->process(imu(t)));
  }
  estimator_->process(imu(t, Eigen::Vector3d(0.0, 0.0, 2.0 * kG)));
  for (; t < 1.2; t += kDt) {
    EXPECT_FALSE(estimator_->process(imu(t)));
  }
  EXPECT_FALSE(estimator_->filter().initialized());    // still waiting for the sensors
  estimator_->process(mag);
  estimator_->process(gnss);
  estimator_->process(baro);
  EXPECT_TRUE(estimator_->process(imu(t)));
  ASSERT_TRUE(estimator_->filter().initialized());
  EXPECT_TRUE(estimator_->filter().position().isApprox(gnss.a));
  EXPECT_NEAR(yaw_of(estimator_->filter().orientation()), yaw, 1e-6);
  EXPECT_NEAR(estimator_->filter().baro_bias(), 5.0, 1e-9);
  EXPECT_NEAR(estimator_->filter().gyro_bias().z(), 0.003, 1e-9);
  // Afterwards each IMU sample gives an estimate
  EXPECT_TRUE(estimator_->process(imu(t + kDt)));
  EXPECT_FALSE(estimator_->process(gnss));
}

TEST_F(InsEkfEstimatorTest, RecoversWhenTheGnssKeepsBeingRejected)
{
  start({0.05, 0.08});     // precise GNSS: a wrong state is soon beyond the gate
  // Start at rest with every sensor
  InsMeasurement gnss;
  gnss.type = InsMeasurement::Type::kGnss;
  gnss.a = Eigen::Vector3d(1.0, 2.0, 0.3);
  InsMeasurement mag;
  mag.type = InsMeasurement::Type::kMag;
  mag.a = Eigen::Vector3d(0.216, 0.0097, 0.429);
  InsMeasurement baro;
  baro.type = InsMeasurement::Type::kBaro;
  baro.scalar = 0.3;
  double t = 0.0;
  for (; !estimator_->filter().initialized(); t += kDt) {
    gnss.stamp = mag.stamp = baro.stamp = t;
    estimator_->process(mag);
    estimator_->process(gnss);
    estimator_->process(baro);
    estimator_->process(imu(t));
  }
  // The IMU sees half of a jolt: the filter thinks it keeps moving down at 5 m/s
  estimator_->process(imu(t, Eigen::Vector3d(0.0, 0.0, kG - 1250.0)));
  t += kDt;
  EXPECT_LT(estimator_->filter().velocity().z(), -4.5);
  // The vehicle is still; the GNSS soon disagrees by more than the gate
  for (int k = 0; t < 6.0; ++k, t += kDt) {
    estimator_->process(imu(t));
    if (k % 25 == 0) {
      gnss.stamp = baro.stamp = t;
      estimator_->process(gnss);
      estimator_->process(baro);
    }
  }
  EXPECT_GE(estimator_->resets(), 1);
  EXPECT_LT((estimator_->filter().position() - gnss.a).norm(), 0.1);
  EXPECT_LT(estimator_->filter().velocity().norm(), 0.1);
}

TEST_F(InsEkfEstimatorTest, PositionIsUnknownWithoutAiding)
{
  InsMeasurement gnss;
  gnss.type = InsMeasurement::Type::kGnss;
  gnss.a = Eigen::Vector3d(1.0, 2.0, 0.3);
  InsMeasurement mag;
  mag.type = InsMeasurement::Type::kMag;
  mag.a = Eigen::Vector3d(0.216, 0.0097, 0.429);
  InsMeasurement baro;
  baro.type = InsMeasurement::Type::kBaro;
  baro.scalar = 0.3;
  double t = 0.0;
  const auto run = [&](double until, bool with_gnss) {
      for (int k = 0; t < until; ++k, t += kDt) {
        if (with_gnss && k % 25 == 0) {
          gnss.stamp = mag.stamp = baro.stamp = t;
          estimator_->process(gnss);
          estimator_->process(mag);
          estimator_->process(baro);
        }
        estimator_->process(imu(t));
      }
    };
  pilotaro::VehicleState estimate;
  run(4.0, true);
  ASSERT_TRUE(estimator_->filter().initialized());
  estimator_->current_estimate(estimate);
  EXPECT_LT(estimate.position_std, 1.0);
  // GNSS lost for longer than aiding_timeout (5 s): position unknown
  run(8.5, false);
  estimator_->current_estimate(estimate);
  EXPECT_GT(estimate.position_std, 0.5);      // dead reckoning: the uncertainty grows...
  EXPECT_LT(estimate.position_std, 100.0);    // ...but it is not marked unknown yet
  run(11.0, false);
  estimator_->current_estimate(estimate);
  EXPECT_GE(estimate.position_std, 100.0);
  // Back: valid again
  run(12.0, true);
  estimator_->current_estimate(estimate);
  EXPECT_LT(estimate.position_std, 1.0);
}

}  // namespace pilotaro_estimators

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
