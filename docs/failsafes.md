# Failsafes and pre-arm checks

Each failsafe has a condition and an action (`failsafe.*` parameters of `states_node`):
`none`, `warn` (only reported), `loiter`, `rtl`, `land` or `disarm`. The strongest active action
is applied. While it lasts, the commands that would undo it (`takeoff`, `offboard`) are
rejected; `landing` is always accepted. The active failsafes are published on `failsafe`
and logged. The parameters can be changed in flight (`ros2 param set`).

| Failsafe | Condition | Default action |
|---|---|---|
| `battery_low` / `battery_critical` | charge ≤ `battery_low` (0.2) / `battery_critical` (0.1) | `warn` / `land` |
| `battery_lost` | battery reported once, then silent for `battery_timeout` (2 s) | `warn` |
| `geofence` | beyond `geofence_radius`, `geofence_max_height` or `geofence_max_depth` from the arming point by more than `geofence_margin` (0.3 m); all off by default | `loiter` inside the fence |
| `navigation` | horizontal uncertainty of the estimate > `max_position_std` (1 m) | `land` level, without horizontal control |
| `heartbeat` | no ground station heartbeat for `heartbeat_timeout` (off by default) | `loiter` |
| `attitude` | tilt > `max_tilt` (1.2 rad) for `max_tilt_time` (0.3 s): flip, lost motor | `disarm` |
| (state estimate lost) | no estimate for `estimate_timeout` | disarm |

- **Geofence:** offboard goals and the takeoff altitude are clamped to it, so the vehicle
  stops at the fence instead of crossing it. The failsafe only acts if it is pushed or flown
  (velocity setpoints) out.
- **Navigation lost:** `InsEkfEstimator` reports an unknown position when it has had no
  GNSS or odometry for `aiding_timeout` (5 s), as EKF2 does. Without a valid horizontal
  estimate, holding a position would chase a drifting estimate, so the vehicle descends
  level, without horizontal control (`ControlSetpoint::xy_velocity = false`), like the
  *Descend* mode of PX4. In simulation, with GNSS cut 5 s after hovering, the x500 lands
  1–3 m from where it was.
- **Underwater vehicles:** `land` takes them to the bottom. `bluerov2_heavy.params.yaml`
  uses `disarm` for battery and navigation, so the positively buoyant vehicle surfaces.

## Pre-arm checks

`arming` answers with every failed check: fresh estimate; tilt below
`max_tilt_arming` (0.35 rad) and angular velocity below `max_rate_arming` (0.3 rad/s);
horizontal uncertainty below `max_position_std`; battery (if reported) fresh and above
`battery_min_arm` (0.3); ground station heartbeat (if enabled).

[Back to the README](../README.md)
