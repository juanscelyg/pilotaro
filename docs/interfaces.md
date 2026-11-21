# ROS 2 interfaces

Every service is a `std_srvs/Trigger` in the robot namespace (`/<robot_name>/...`); a rejected command answers `success: false` with the reason. What each mode does is in [Flight modes](modes.md).

| Service | Accepted in | Does |
|---|---|---|
| `arming` | `DISARMED` passing the pre-arm checks, or `ARMED` | Arms (stores the arming point) or disarms. |
| `takeoff` | `ARMED`, no failsafe holding or landing the vehicle | Goes to `TAKEOFF`. |
| `loiter` | flying | Holds the current position and heading. |
| `landing` | flying | Goes to `LANDING`. |
| `offboard` | any mode but `DISARMED`, with a setpoint newer than `offboard_timeout` and no failsafe holding or landing the vehicle | Goes to `OFFBOARD`. |
| `mission` | `ARMED` (takes off first) or flying, with a mission inside the geofence and no failsafe holding or landing the vehicle | Goes to `MISSION`. |
| `rtl` | flying | Goes to `RTL`. |

| Topic | Type | |
|---|---|---|
| `mode` | `std_msgs/String` | Current mode (transient local: late subscribers get it). |
| `offboard/pose` | `geometry_msgs/PoseStamped` | Position and yaw in the world frame. With `offboard_frame_id` set, other frames are ignored. |
| `offboard/twist` | `geometry_msgs/Twist` | Linear velocity in the world frame and yaw rate (`angular.z`). |
| `estimate` | `nav_msgs/Odometry` | State estimate used by the controller: pose in the world frame, twist in the body frame (REP 147), stamp of the estimate; the horizontal uncertainty in the pose covariance. |
| `mission` | `nav_msgs/Path` | Waypoints in the world frame (position, and yaw if the orientation is set). Transient local: publish it once. Not replaced while it is being flown. |
| `mission/current` | `std_msgs/Int32` | Waypoint being flown (−1 outside a mission); transient local. |
| `battery` | `sensor_msgs/BatteryState` | Input: `percentage` (0–1) for the battery failsafes and pre-arm check. |
| `heartbeat` | `std_msgs/Empty` | Input: ground station heartbeat, if `failsafe.heartbeat_timeout` is set. |
| `failsafe` | `std_msgs/String` | Active failsafes, comma separated (empty: none); transient local. |

Offboard setpoints must keep arriving (more often than every `offboard_timeout`, 0.5 s by
default); if they stop, the vehicle holds its position in `LOITER`. Touchdown is detected
when the vehicle stays within `land_detect_altitude` of the arming point, slower than
`land_detect_speed`, for `land_detect_time`. All these parameters belong to `states_node`
in `params/pilotaro.params.yaml` (see [Parameters](parameters.md)).

[Back to the README](../README.md)
