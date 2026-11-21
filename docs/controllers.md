# Controllers

The controller plugin is selected with `controller.plugin` in the `controllers_node` block (see [Architecture and plugins](architecture.md)). It runs only while flying, on every new estimate.

## `pilotaro_controllers/CascadeController`

Cascade controller for multirotors: position P → velocity PI → thrust → SO(3) attitude
PD → allocation. Allocation uses the pseudoinverse, so it supports any planar geometry with 4 or
more rotors (there is a test with a hexarotor). Its gains and limits are in the `controller`
block. It only accepts rotors that push towards +z and are not reversible.

- **Yaw within the rotor authority:** the yaw error commands a yaw rate limited to
  `max_yaw_rate` (1 rad/s, 0 = no limit); while not limited it is the same PD law.
- **Desaturation by priorities:** when the rotors cannot give the whole wrench, roll and
  pitch come first (the collective thrust moves to make them fit, and they are scaled down
  only if the difference between rotors does not fit), then as much yaw torque as fits in
  the remaining margin. A large yaw step no longer takes thrust or attitude away: on the
  x500, the time with saturated motors in a 90° yaw went from 0.37 s to 0.10 s.

## Features of both controllers

- **Filtered measurements:** first-order low-pass filters on the measured velocity
  (`velocity_cutoff`, 10 Hz) and angular velocity (`angular_rate_cutoff`, 10 Hz), and
  optionally on the position (`position_cutoff`, 0 = off). Estimation noise would otherwise
  reach the actuators: with estimator-like noise they cut the motor activity by two thirds.
  At 5 Hz the velocity filter delayed the settling (±5 cm in 3.0 s instead of 2.2 s).
- **Straight-line approach:** with a position setpoint on the 3 axes, the velocity setpoint
  points at the goal and the whole vector is scaled down to the speed limits (and to
  `ControlSetpoint::max_speed`, used by missions), so the vehicle moves along the straight
  line to it. Within 1 m of the goal the gain blends back into the gain of each axis, so
  every axis converges at its own rate.

## `pilotaro_controllers/UnderwaterController`

For vehicles with vectored, reversible thrusters (ROV, AUV). Controls 6 DOF:

- **Position and velocity:** position P and velocity PI in the world frame.
- **Force:** computed with the mass plus added mass, with feed-forward of linear and
  quadratic damping at the setpoint velocity and of the net buoyancy.
- **Attitude:** roll and pitch to zero and the setpoint yaw, with an SO(3) PD that uses the
  inertia plus added inertia.
- **Allocation:** the wrench is distributed among the thrusters with the pseudoinverse of the
  thruster matrix (position, axis and reaction torque of each one).
- **Uncontrollable DOF:** selected automatically, by priority (surge, sway, heave, yaw, roll,
  pitch). The 8 thrusters of the BlueROV2 Heavy actuate all 6 DOF; on a vehicle where one
  is not actuated (e.g. pitch on a 6-thruster BlueROV2) it is left to its hydrostatic
  stability and the log says so (`not actuated: pitch`).
- **Saturation:** if a thruster saturates, all of them are scaled together to keep the
  direction of the wrench.

Parameters (prefix `controller.`): `position_p`, `velocity_p`, `velocity_i`,
`velocity_i_max`, `attitude_p`, `angular_rate_p`, `max_horizontal_speed`,
`max_vertical_speed`, `added_mass`, `added_inertia`, `linear_damping`, `quadratic_damping`,
`net_buoyancy`, `position_cutoff`, `velocity_cutoff` and `angular_rate_cutoff`.

The gains in the main file are the multirotor ones, and both controllers share names
(`position_p`, `velocity_p`…). So when a vehicle file switches controller, it must set all
of its parameters, as `config/bluerov2_heavy.params.yaml` does.

[Back to the README](../README.md)
