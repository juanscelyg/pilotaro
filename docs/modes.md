# Flight modes

`states_node` runs the flight modes. They are commanded with the services of [the ROS 2 interface](interfaces.md); the failsafes can also change them ([Failsafes](failsafes.md)).

```text
DISARMED --arming--> ARMED --takeoff--> TAKEOFF --(altitude reached)--> LOITER
ARMED --arming--> DISARMED
any but DISARMED --offboard--> OFFBOARD --(setpoint timeout)--> LOITER
flying --loiter--> LOITER          flying --landing--> LANDING --(touchdown)--> DISARMED
armed --(no state estimate for estimate_timeout)--> DISARMED
flying --(failsafe)--> LOITER, RTL, LANDING or DISARMED (see Failsafes)
ARMED --mission--> TAKEOFF --> MISSION;  flying --mission--> MISSION
MISSION --(last waypoint)--> LOITER, RTL or LANDING
flying --rtl--> RTL --(over the arming point)--> LANDING, DISARMED or LOITER
```

| Mode | Vehicle |
|---|---|
| `DISARMED` | Motors stopped. |
| `ARMED` | On the ground (or at the surface), motors at `idle_rot_velocity`. |
| `TAKEOFF` | Climbs `takeoff_altitude` from the arming point at `takeoff_speed` (dives if negative), accelerating and braking at `takeoff_acceleration`; the setpoint never leads the vehicle by more than `takeoff_max_lead`. |
| `LOITER` | Holds its position and heading. |
| `LANDING` | Descends at `landing_speed` keeping XY; disarms on touchdown. |
| `OFFBOARD` | Follows the `offboard/pose` or `offboard/twist` setpoints. |
| `MISSION` | Flies the waypoints of the `mission` topic in order (see [Missions](#missions-and-return-to-launch)). |
| `RTL` | Returns to the arming point: climbs to `mission.rtl_altitude` over it if lower, flies back at `mission.rtl_speed` and then lands (`mission.rtl_final_action`). |

`TAKEOFF`, `LOITER`, `LANDING`, `OFFBOARD`, `MISSION` and `RTL` are the flying modes.

## Missions and return to launch

A mission is a list of waypoints (`nav_msgs/Path` on `mission`); the `mission` service flies
it, taking off first if the vehicle is on the ground. The setpoint travels each leg in a
straight line at `mission.speed`, accelerating and braking at `mission.acceleration`, with
the profile acceleration as feed-forward (`ControlSetpoint::acceleration`, jerk limited to
3 m/s³) and the commanded speed capped at it (`ControlSetpoint::max_speed`). It waits for a
vehicle that lags more than `mission.max_lead`, and consecutive legs share their corner, so
the path stays on the lines between waypoints. A waypoint is reached within
`mission.acceptance_radius` and below `mission.acceptance_speed`, after
`mission.hold_time`. After the last one
`mission.end_action` applies: `loiter` (default), `rtl` or `land`.

`rtl` (service, end of mission or failsafe action) climbs to `mission.rtl_altitude` over the
arming point if lower (never descends; 0 returns at the current height), flies back at
`mission.rtl_speed` and then applies `mission.rtl_final_action`: `land` (default, on the
arming point itself), `disarm` or `loiter`. The BlueROV2 Heavy returns at its depth and disarms, surfacing by its buoyancy.
A mission with a waypoint outside the geofence is rejected, and without a valid position an
`rtl` failsafe lands instead.

```bash
ros2 topic pub --once /x500/mission nav_msgs/msg/Path "{header: {frame_id: odom}, poses: [
  {pose: {position: {x: 4.0, y: 0.0, z: 3.0}}},
  {pose: {position: {x: 4.0, y: 4.0, z: 3.0}, orientation: {z: 0.707, w: 0.707}}},
  {pose: {position: {x: 0.0, y: 4.0, z: 2.0}}}]}" --qos-durability transient_local
ros2 service call /x500/arming std_srvs/srv/Trigger
ros2 service call /x500/mission std_srvs/srv/Trigger     # takes off and flies it
ros2 service call /x500/rtl std_srvs/srv/Trigger         # or return at any time
```

[Back to the README](../README.md)
