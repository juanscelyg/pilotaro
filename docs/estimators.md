# State estimators

The estimator plugin is selected with `estimator.plugin` in the `estimator_node` block (see [Architecture and plugins](architecture.md)). Its output is also published on `estimate` ([interfaces](interfaces.md)).

## `pilotaro_estimators/OdometryEstimator`

Takes the state from a `nav_msgs/Odometry` (in simulation, the *ground truth* odometry of
the `OdometryPublisher` at 250 Hz):

| Parameter | Default | |
|---|---|---|
| `topic` | `odometry` | Relative to the robot namespace. |
| `twist_in_body_frame` | `true` | Twist in the child frame (REP 147). `false`: linear velocity in world. |
| `frame_id` | `""` | If set, odometry with a different `header.frame_id` is ignored. |
| `use_message_stamp` | `true` | `false`: stamps the estimate with the reception time. |

The rest of the system runs at the pace of the estimates: the controller only runs on a new
estimate, and its `dt` is the difference between their stamps. If no estimates arrive for
`estimate_timeout` while the vehicle is armed, it disarms.

## `pilotaro_estimators/InsEkfEstimator`

Inertial navigation with the vehicle's own sensors: an error-state EKF (16 states: position,
velocity, attitude, accelerometer and gyro biases, barometer offset) that integrates the IMU
and corrects it with GNSS, barometer, magnetometer and, optionally, an external odometry
(VIO, DVL + USBL, motion capture...). It gives an estimate on every IMU sample.

- **Start:** once the vehicle has been still for `init_time`. Roll and pitch come from the
  accelerometer, the gyro bias from the mean gyro, the yaw from the magnetometer (or the
  odometry), the position from the first GNSS fix (or the odometry).
- **Magnetometer:** the direction of the field is fused in 3D (body = Rᵀ·world), which
  keeps tilt and heading consistent; only its horizontal direction (`mag_heading`) has to
  be given, its inclination is measured at the start.
- **Outliers and recovery:** measurements beyond `innovation_gate` sigmas are rejected. If a
  sensor keeps being rejected for `reset_timeout`, its state (position, velocity or
  barometer offset) is reset to it, as EKF2 does. This recovers, for instance, from a
  touchdown the IMU did not sample.
- **Cost:** about 0.2 % of a core at 250 Hz (Release build).

| Parameter | Default | |
|---|---|---|
| `imu_topic`, `gnss_topic`, `baro_topic`, `mag_topic` | `imu`, `navsat`, `air_pressure`, `magnetometer` | Empty disables the sensor (except the IMU). |
| `gnss_velocity_topic`, `odometry_topic` | `""` | GNSS velocity (`TwistWithCovarianceStamped`, ENU) and external odometry. |
| `gnss_origin` | `[]` | `[lat, lon, alt]` of the world origin; empty: the first fix. |
| `gnss_noise`, `gnss_velocity_noise` | `[0.5, 1.0]` m, `0.1` m/s | Horizontal and vertical. |
| `baro_noise`, `sea_level_pressure` | `0.5` m, `101325` Pa | |
| `mag_heading`, `mag_noise` | `0` rad, `0.05` | Direction of the horizontal field from world x (π/2 − declination in ENU); noise of the unit field vector. |
| `odometry_position_noise`, `odometry_velocity_noise`, `odometry_attitude_noise`, `odometry_yaw_noise` | `0.05` m, `0`, `0`, `0` | 0 = not fused. |
| `gyro_noise`, `accel_noise` | `1e-3`, `4e-3` | Process noise densities (rad/s/√Hz, m/s²/√Hz). |
| `gyro_bias_noise`, `accel_bias_noise`, `baro_bias_noise` | `1e-5`, `1e-4`, `5e-3` | Random walk of the biases. |
| `innovation_gate`, `reset_timeout`, `init_time` | `5` σ, `1` s, `1` s | |

```yaml
/**/estimator_node:
  ros__parameters:
    estimator:
      plugin: "pilotaro_estimators/InsEkfEstimator"
      gnss_origin: [40.2849, -3.8211, 650.0]   # spherical_coordinates of the world
      mag_heading: 0.0448                      # horizontal field of the gz world, from x
```

[Back to the README](../README.md)
