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
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "pilotaro_estimators/InsEkfEstimator.hpp"

namespace pilotaro_estimators
{

namespace
{
double stamp_of(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

Eigen::Vector3d to_eigen(const geometry_msgs::msg::Vector3 & v)
{
  return {v.x, v.y, v.z};
}

double yaw_of(const Eigen::Quaterniond & q)
{
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

// Position uncertainty reported without aiding [m]
constexpr double kUnknownPositionStd = 1e3;
// Max time between IMU samples that is integrated [s]
constexpr double kMaxImuGap = 0.1;
// Accelerometer and gyro bounds to consider the vehicle still
constexpr double kStillAccel = 0.5;    // |  |f| - g | [m/s^2]
constexpr double kStillGyro = 0.1;     // |w| [rad/s]
}  // namespace

Eigen::Vector3d
geodetic_to_enu(
  double latitude, double longitude, double altitude, const Eigen::Vector3d & origin)
{
  constexpr double a = 6378137.0;
  constexpr double e2 = 6.69437999014e-3;
  const double lat0 = origin.x() * M_PI / 180.0;
  const double s = std::sin(lat0);
  const double w = 1.0 - e2 * s * s;
  const double meridian = a * (1.0 - e2) / (w * std::sqrt(w));
  const double normal = a / std::sqrt(w);
  const double north = (latitude - origin.x()) * M_PI / 180.0 * (meridian + origin.z());
  const double east = (longitude - origin.y()) * M_PI / 180.0 * (normal + origin.z()) *
    std::cos(lat0);
  return {east, north, altitude - origin.z()};
}

double
pressure_to_height(double pressure, double sea_level_pressure)
{
  return 44330.77 * (1.0 - std::pow(pressure / sea_level_pressure, 0.190263));
}

double
magnetic_yaw(
  const Eigen::Vector3d & field_body, const Eigen::Quaterniond & orientation, double heading)
{
  // R = Rz(yaw) * R_tilt: field_body = R^T * field_world, so R_tilt * field_body is the
  // world field turned by -yaw
  const double yaw = yaw_of(orientation);
  const Eigen::Vector3d levelled =
    Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * (orientation * field_body);
  return std::atan2(
    std::sin(heading - std::atan2(levelled.y(), levelled.x())),
    std::cos(heading - std::atan2(levelled.y(), levelled.x())));
}

bool
InsEkfEstimator::on_initialize()
{
  const auto imu_topic = declare_parameter<std::string>("imu_topic", "imu");
  const auto gnss_topic = declare_parameter<std::string>("gnss_topic", "navsat");
  const auto gnss_velocity_topic = declare_parameter<std::string>("gnss_velocity_topic", "");
  const auto baro_topic = declare_parameter<std::string>("baro_topic", "air_pressure");
  const auto mag_topic = declare_parameter<std::string>("mag_topic", "magnetometer");
  const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "");

  const auto origin = declare_parameter<std::vector<double>>("gnss_origin", {});
  if (origin.size() == 3) {
    origin_ = Eigen::Vector3d(origin[0], origin[1], origin[2]);
  } else if (!origin.empty()) {
    RCLCPP_ERROR(get_logger(), "gnss_origin must be [latitude, longitude, altitude] or empty");
    return false;
  }
  const auto gnss_noise = declare_parameter<std::vector<double>>("gnss_noise", {0.5, 1.0});
  if (gnss_noise.size() != 2) {
    RCLCPP_ERROR(get_logger(), "gnss_noise must be [horizontal, vertical]");
    return false;
  }
  gnss_sigma_ = {gnss_noise[0], gnss_noise[0], gnss_noise[1]};
  gnss_velocity_sigma_ = declare_parameter("gnss_velocity_noise", gnss_velocity_sigma_);
  baro_sigma_ = declare_parameter("baro_noise", baro_sigma_);
  sea_level_pressure_ = declare_parameter("sea_level_pressure", sea_level_pressure_);
  mag_heading_ = declare_parameter("mag_heading", mag_heading_);
  mag_sigma_ = declare_parameter("mag_noise", mag_sigma_);
  odometry_position_sigma_ = declare_parameter("odometry_position_noise", odometry_position_sigma_);
  odometry_velocity_sigma_ = declare_parameter("odometry_velocity_noise", odometry_velocity_sigma_);
  odometry_yaw_sigma_ = declare_parameter("odometry_yaw_noise", odometry_yaw_sigma_);
  odometry_attitude_sigma_ =
    declare_parameter("odometry_attitude_noise", odometry_attitude_sigma_);
  gate_ = declare_parameter("innovation_gate", gate_);
  init_time_ = declare_parameter("init_time", init_time_);
  reset_timeout_ = declare_parameter("reset_timeout", reset_timeout_);
  aiding_timeout_ = declare_parameter("aiding_timeout", aiding_timeout_);

  InsEkfParams p;
  p.gyro_noise = declare_parameter("gyro_noise", p.gyro_noise);
  p.accel_noise = declare_parameter("accel_noise", p.accel_noise);
  p.gyro_bias_noise = declare_parameter("gyro_bias_noise", p.gyro_bias_noise);
  p.accel_bias_noise = declare_parameter("accel_bias_noise", p.accel_bias_noise);
  p.baro_bias_noise = declare_parameter("baro_bias_noise", p.baro_bias_noise);
  p.gravity = get_vehicle().gravity;
  filter_ = InsEkf(p);

  if (imu_topic.empty()) {
    RCLCPP_ERROR(get_logger(), "imu_topic is required");
    return false;
  }
  use_gnss_ = !gnss_topic.empty();
  use_baro_ = !baro_topic.empty();
  use_mag_ = !mag_topic.empty();
  use_odometry_ = !odometry_topic.empty();
  if (!use_gnss_ && !use_odometry_) {
    RCLCPP_WARN(get_logger(), "No GNSS or odometry: the position will drift");
  }
  if (!use_mag_ && !odometry_heading()) {
    RCLCPP_WARN(get_logger(), "No magnetometer or odometry yaw: the yaw will drift");
  }

  auto node = get_node();
  const auto qos = rclcpp::SensorDataQoS();
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      InsMeasurement m;
      m.type = InsMeasurement::Type::kImu;
      m.stamp = stamp_of(msg->header.stamp);
      m.a = to_eigen(msg->linear_acceleration);
      m.b = to_eigen(msg->angular_velocity);
      push(m);
    });
  if (use_gnss_) {
    gnss_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic, qos, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
        !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude))
        {
          return;
        }
        if (!origin_) {
          origin_ = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
          RCLCPP_INFO(
            get_logger(), "GNSS origin from the first fix: %.7f, %.7f, %.2f m",
            msg->latitude, msg->longitude, msg->altitude);
        }
        InsMeasurement m;
        m.type = InsMeasurement::Type::kGnss;
        m.stamp = stamp_of(msg->header.stamp);
        m.a = geodetic_to_enu(msg->latitude, msg->longitude, msg->altitude, *origin_);
        push(m);
      });
  }
  if (!gnss_velocity_topic.empty()) {
    gnss_velocity_sub_ = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      gnss_velocity_topic, qos,
      [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
        InsMeasurement m;
        m.type = InsMeasurement::Type::kGnssVelocity;
        m.stamp = stamp_of(msg->header.stamp);
        m.a = to_eigen(msg->twist.twist.linear);
        push(m);
      });
  }
  if (use_baro_) {
    baro_sub_ = node->create_subscription<sensor_msgs::msg::FluidPressure>(
      baro_topic, qos, [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
        InsMeasurement m;
        m.type = InsMeasurement::Type::kBaro;
        m.stamp = stamp_of(msg->header.stamp);
        m.scalar = pressure_to_height(msg->fluid_pressure, sea_level_pressure_);
        push(m);
      });
  }
  if (use_mag_) {
    mag_sub_ = node->create_subscription<sensor_msgs::msg::MagneticField>(
      mag_topic, qos, [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
        InsMeasurement m;
        m.type = InsMeasurement::Type::kMag;
        m.stamp = stamp_of(msg->header.stamp);
        m.a = to_eigen(msg->magnetic_field);
        push(m);
      });
  }
  if (use_odometry_) {
    odometry_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        InsMeasurement m;
        m.type = InsMeasurement::Type::kOdometry;
        m.stamp = stamp_of(msg->header.stamp);
        const auto & pose = msg->pose.pose;
        m.a = {pose.position.x, pose.position.y, pose.position.z};
        m.q = Eigen::Quaterniond(
          pose.orientation.w, pose.orientation.x, pose.orientation.y,
          pose.orientation.z).normalized();
        m.b = m.q * to_eigen(msg->twist.twist.linear);   // REP 147: twist in body frame
        push(m);
      });
  }
  RCLCPP_INFO(
    get_logger(), "INS EKF on %s%s%s%s%s", imu_sub_->get_topic_name(),
    use_gnss_ ? " + GNSS" : "", use_baro_ ? " + baro" : "", use_mag_ ? " + mag" : "",
    use_odometry_ ? " + odometry" : "");
  return true;
}

void
InsEkfEstimator::push(const InsMeasurement & m)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.push_back(m);
}

bool
InsEkfEstimator::update(pilotaro::VehicleState & estimate)
{
  // The two buffers swap roles: no allocation once they have grown
  batch_.clear();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    batch_.swap(queue_);
  }
  std::stable_sort(
    batch_.begin(), batch_.end(),
    [](const InsMeasurement & a, const InsMeasurement & b) {return a.stamp < b.stamp;});
  bool fresh = false;
  for (const auto & m : batch_) {
    fresh = process(m) || fresh;
  }
  if (!fresh) {
    return false;
  }
  current_estimate(estimate);
  return true;
}

void
InsEkfEstimator::current_estimate(pilotaro::VehicleState & estimate)
{
  estimate.position = filter_.position();
  estimate.velocity = filter_.velocity();
  estimate.orientation = filter_.orientation();
  estimate.angular_velocity = last_gyro_ - filter_.gyro_bias();
  estimate.stamp = last_imu_stamp_;
  const auto & P = filter_.covariance();
  estimate.position_std = std::sqrt(P(InsEkf::kP, InsEkf::kP) + P(InsEkf::kP + 1, InsEkf::kP + 1));
  // Without position aiding for aiding_timeout the inertial position is not trusted (as
  // EKF2_NOAID_TOUT): report it as unknown so the navigation failsafe can act
  if ((use_gnss_ || use_odometry_) && aiding_timeout_ > 0.0) {
    const double aided = std::max(
      accepted_[static_cast<int>(InsMeasurement::Type::kGnss)],
      accepted_[static_cast<int>(InsMeasurement::Type::kOdometry)]);
    const bool lost = last_imu_stamp_ - aided > aiding_timeout_;
    if (lost != aiding_lost_) {
      aiding_lost_ = lost;
      if (lost) {
        RCLCPP_WARN(
          get_logger(), "INS EKF: no position aiding for %.1f s, position not valid",
          aiding_timeout_);
      } else {
        RCLCPP_INFO(get_logger(), "INS EKF: position aiding back");
      }
    }
    if (lost) {
      estimate.position_std = std::max(estimate.position_std, kUnknownPositionStd);
    }
  }
}

bool
InsEkfEstimator::process(const InsMeasurement & m)
{
  using Type = InsMeasurement::Type;
  // Last value of each aiding sensor, for the initialization
  switch (m.type) {
    case Type::kGnss: last_position_ = m.a; break;
    case Type::kBaro: last_baro_ = m.scalar; break;
    case Type::kMag: last_field_ = m.a; break;
    case Type::kOdometry:
      if (!use_gnss_) {
        last_position_ = m.a;
      }
      last_odometry_orientation_ = m.q;
      break;
    default: break;
  }
  if (!filter_.initialized()) {
    return try_initialize(m);
  }

  switch (m.type) {
    case Type::kImu: {
        const double dt = m.stamp - last_imu_stamp_;
        if (dt <= 0.0) {
          return false;
        }
        last_gyro_ = m.b;
        last_imu_stamp_ = m.stamp;
        if (dt > kMaxImuGap) {
          return false;      // not integrated, the next sample continues
        }
        filter_.predict(m.a, m.b, dt);
        return true;
      }
    case Type::kGnss:
      fuse_or_reset(
        m.type, m.stamp, [&] {return filter_.update_position(m.a, gnss_sigma_, gate_);},
        [&] {filter_.reset_position(m.a, gnss_sigma_);});
      return false;
    case Type::kGnssVelocity: {
        const Eigen::Vector3d sigma = Eigen::Vector3d::Constant(gnss_velocity_sigma_);
        fuse_or_reset(
          m.type, m.stamp, [&] {return filter_.update_velocity(m.a, sigma, gate_);},
          [&] {filter_.reset_velocity(m.a, sigma);});
        return false;
      }
    case Type::kBaro: {
        // With GNSS the barometer offset is what drifts; without it (or with the GNSS
        // lost) the barometer is the height reference and the height is reset instead
        const bool gnss_height = use_gnss_ &&
          m.stamp - accepted_[static_cast<int>(Type::kGnss)] < reset_timeout_;
        fuse_or_reset(
          m.type, m.stamp, [&] {return filter_.update_baro(m.scalar, baro_sigma_, gate_);},
          [&] {
            if (gnss_height) {
              filter_.reset_baro_bias(m.scalar, baro_sigma_);
            } else {
              filter_.reset_height(m.scalar, baro_sigma_);
            }
          });
        return false;
      }
    case Type::kMag:
      if (m.a.norm() > 0.0) {
        filter_.update_direction(m.a, mag_world_, mag_sigma_, gate_);
      }
      return false;
    case Type::kOdometry: {
        const Eigen::Vector3d sigma = Eigen::Vector3d::Constant(odometry_position_sigma_);
        fuse_or_reset(
          m.type, m.stamp, [&] {return filter_.update_position(m.a, sigma, gate_);},
          [&] {filter_.reset_position(m.a, sigma);});
        if (odometry_velocity_sigma_ > 0.0) {
          filter_.update_velocity(
            m.b, Eigen::Vector3d::Constant(odometry_velocity_sigma_), gate_);
        }
        if (odometry_attitude_sigma_ > 0.0) {
          filter_.update_orientation(m.q, odometry_attitude_sigma_, gate_);
        } else if (odometry_yaw_sigma_ > 0.0) {
          filter_.update_yaw(yaw_of(m.q), odometry_yaw_sigma_, gate_);
        }
        return false;
      }
  }
  return false;
}

template<typename Update, typename Reset>
bool
InsEkfEstimator::fuse_or_reset(
  InsMeasurement::Type type, double stamp, Update update, Reset reset)
{
  double & accepted = accepted_[static_cast<int>(type)];
  if (update()) {
    accepted = stamp;
    return true;
  }
  if (reset_timeout_ <= 0.0 || stamp - accepted < reset_timeout_) {
    return false;
  }
  reset();
  accepted = stamp;
  ++resets_;
  static const char * names[] = {"IMU", "GNSS", "GNSS velocity", "barometer", "magnetometer",
    "odometry"};
  RCLCPP_WARN(
    get_logger(), "INS EKF: %s rejected for %.1f s, state reset to it (NIS %.0f)",
    names[static_cast<int>(type)], reset_timeout_, filter_.last_nis());
  return true;
}

bool
InsEkfEstimator::try_initialize(const InsMeasurement & m)
{
  if (m.type != InsMeasurement::Type::kImu) {
    return false;
  }
  const double g = get_vehicle().gravity;
  const bool still = std::abs(m.a.norm() - g) < kStillAccel && m.b.norm() < kStillGyro;
  if (!still || still_since_ < 0.0) {
    still_since_ = m.stamp;
    accel_sum_.setZero();
    gyro_sum_.setZero();
    still_samples_ = 0;
    if (!still) {
      return false;
    }
  }
  accel_sum_ += m.a;
  gyro_sum_ += m.b;
  ++still_samples_;
  last_imu_stamp_ = m.stamp;
  last_gyro_ = m.b;
  if (m.stamp - still_since_ < init_time_) {
    return false;
  }
  const bool need_mag = use_mag_ && !odometry_heading();
  if ((use_gnss_ || use_odometry_) && !last_position_) {
    return false;
  }
  if ((need_mag && !last_field_) || (use_baro_ && !last_baro_) ||
    (!need_mag && odometry_heading() && !last_odometry_orientation_))
  {
    return false;
  }

  // Roll and pitch from the mean specific force, yaw from the magnetometer or odometry
  const Eigen::Vector3d f = accel_sum_ / still_samples_;
  const double roll = std::atan2(f.y(), f.z());
  const double pitch = std::atan2(-f.x(), std::hypot(f.y(), f.z()));
  Eigen::Quaterniond tilt = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  double yaw = 0.0;
  if (need_mag) {
    yaw = magnetic_yaw(*last_field_, tilt, mag_heading_);
  } else if (last_odometry_orientation_) {
    yaw = yaw_of(*last_odometry_orientation_);
  }
  const Eigen::Quaterniond orientation =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * tilt;
  if (need_mag || last_field_) {
    // World field direction: heading from the parameter, inclination as measured now
    mag_world_ = (orientation * last_field_.value_or(Eigen::Vector3d::UnitX())).normalized();
  }
  const Eigen::Vector3d position = last_position_.value_or(Eigen::Vector3d::Zero());

  InsEkfSigmas sigmas;
  sigmas.position = use_gnss_ ? gnss_sigma_.maxCoeff() : odometry_position_sigma_;
  sigmas.yaw = need_mag ? 0.05 : 0.1;
  sigmas.gyro_bias = 0.002;
  filter_.initialize(
    position, orientation, gyro_sum_ / still_samples_, last_baro_.value_or(position.z()),
    sigmas);
  for (double & a : accepted_) {
    a = m.stamp;
  }
  RCLCPP_INFO(
    get_logger(), "INS EKF started: position (%.2f, %.2f, %.2f) m, roll %.1f, pitch %.1f, "
    "yaw %.1f deg", position.x(), position.y(), position.z(), roll * 180.0 / M_PI,
    pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
  return true;
}

void
InsEkfEstimator::reset()
{
  // The navigation state survives disarming; only the pending samples are dropped
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

}  // namespace pilotaro_estimators

PLUGINLIB_EXPORT_CLASS(pilotaro_estimators::InsEkfEstimator, pilotaro::StateEstimatorBase)
