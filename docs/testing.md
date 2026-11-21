# Testing and CI

- **Unit tests:**

  ```bash
  colcon test --packages-select pilotaro_common pilotaro_estimators pilotaro_controllers \
    pilotaro_states pilotaro_actuators pilotaro_description
  ```

  They cover:
  - types and estimators: the INS EKF following a flight with biased, noisy sensors,
    learning the biases at rest, rejecting outliers and recovering from a state the GNSS
    keeps contradicting, the 3D magnetometer and orientation updates, and its start-up;
  - both controllers: the multirotor one with the Bebop 2 parameters and a hexarotor; the
    underwater one with the BlueROV2 Heavy (all 6 DOF) and a 6-thruster BlueROV2 (pitch not
    actuated) geometry, checking the actuated DOF, buoyancy feed-forward, rotation to the
    body frame and saturation; the multirotor desaturation (thrust and roll kept under an
    excessive yaw torque) and the yaw rate limit;
  - the measurement filters and the straight-line velocity setpoint (`test_shaping`);
  - the mode state machine, including diving and the takeoff profile (acceleration, braking
    and no lead over a vehicle still on the ground), the pre-arm checks and every failsafe
    (battery, geofence and depth limit, navigation, ground station link, flip), missions
    (from the ground, straight legs at the mission speed, geofence) and return to launch;
  - the output plugins;
  - URDF ↔ parameters consistency for each vehicle.
- **Flight test** ([`flight_check.py`](../pilotaro_gazebo/scripts/flight_check.py)): flies all
  the given robots at once and checks each step against a criterion:
  1. takeoff is rejected if not armed;
  2. arms, takes off and reaches LOITER at the takeoff altitude;
  3. holds position in hover without saturating the motors;
  4. in offboard, reaches a 3D goal (measuring overshoot) and a yaw change;
  5. lands and disarms.

  For underwater vehicles there are two optional physical checks:
  - `--expect-float Z`: disarmed, it floats still at the surface at z = Z. The check first
    waits (up to `--settle-time`, 60 s) until the vehicle stops bobbing after the spawn;
  - `--expect-ascent V`: after disarming on the bottom, it rises on its own at the terminal
    velocity V given by its net buoyancy and damping.

  Exits with 0 if everything passes.

  ```bash
  ros2 launch pilotaro_gazebo sim.launch.py headless:=true cameras:=false \
    robots:=big:x500,small:bebop2 spacing:=3.0
  ros2 run pilotaro_gazebo flight_check.py big small
  ```

  Simulation results, with both flying at the same time:

  | | x500 | bebop2 |
  |---|---|---|
  | Altitude after takeoff | 2.229 m (target 2.227) | 2.044 m (target 2.051) |
  | Hover drift (5 s) | 2.1 cm | 0.5 cm |
  | Rotors in hover | 769 rad/s | 819 / 818 / 792 / 793 rad/s |
  | Offboard goal (2, 1, 1) m: error / overshoot | 0.6 cm / 1.4 cm | 3.2 cm / 7.0 cm |
  | Yaw 90° | 0.0° | 0.0° |
  | Landing | DISARMED | DISARMED |

  BlueROV2 Heavy in the underwater world:

  ```bash
  ros2 launch pilotaro_gazebo sim.launch.py headless:=true world:=underwater robots:=rov1:bluerov2_heavy
  ros2 run pilotaro_gazebo flight_check.py rov1 --takeoff-altitude -2.0 --goal-time 15 \
    --landing-time 80 --max-rotor-speed 157.7 --expect-float -0.0732 --expect-ascent 0.0412
  ```

  Expected values, computed by hand:
  - **floating:** z = −0.0732 m, the submerged volume (13.4 dm³ of the 13.5 dm³ box) that
    balances the weight;
  - **passive ascent:** 19.0·w² + 23.0·w = 0.98 N gives w = 0.0412 m/s.

  | | bluerov2_heavy |
  |---|---|
  | Floating, disarmed | z = −0.073 m (expected −0.073), still |
  | Dive to 2 m | z = −2.067 m (target −2.073) |
  | Depth-hold drift (5 s) | 0.4 cm |
  | Offboard goal (2, 1, 1) m: error / overshoot | 0.2 cm / 0.0 cm |
  | Yaw 90° | 0.0° |
  | Descent to the bottom | DISARMED at z = −9.97 m |
  | Passive ascent | 0.041 m/s (expected 0.041) |

- **Mission test** ([`mission_check.py`](../pilotaro_gazebo/scripts/mission_check.py)): from
  the ground, an x500 flies three waypoints and returns to launch (`mission.end_action`
  `rtl`). Measured with the ground truth: closest approach 6–11 cm to each waypoint,
  cross-track error up to 0.22 m on its short 4 m legs, top speed 2.16 m/s for a mission
  speed of 2 m/s, climb to the return height and touchdown on the arming point (0.00 m).
  On a 10 m square, the same mission flown by PX4 1.17 passes 13–26 cm from the waypoints
  and takes 28 s, against 3–4 cm and 21 s for pilotaro.

  ```bash
  ros2 launch pilotaro_gazebo sim.launch.py headless:=true robots:=uav1:x500
  ros2 run pilotaro_gazebo mission_check.py uav1     # 14/14 checks passed
  ```

- **Failsafe test** ([`failsafe_check.py`](../pilotaro_gazebo/scripts/failsafe_check.py)):
  injects failures into one robot, setting the failsafe parameters at run time: arming
  rejected with a low battery; an offboard goal beyond a 3 m geofence stops at the fence;
  when the ground station heartbeats stop the vehicle loiters, rejects offboard and reports
  the failsafe, which clears when they come back; with a critical battery it lands and
  disarms, rejecting takeoff and offboard meanwhile.

  ```bash
  ros2 launch pilotaro_gazebo sim.launch.py headless:=true robots:=uav1:x500
  ros2 run pilotaro_gazebo failsafe_check.py uav1     # 13/13 checks passed
  ```

## Continuous integration

[`.github/workflows/rolling.yml`](../.github/workflows/rolling.yml) checks that pilotaro works on
ROS 2 Rolling. It runs on every push and pull request to `rolling`, every Monday (Rolling
changes daily) and on demand (*Actions → rolling → Run workflow*). In a `ros:rolling`
container, without a GPU, it:

1. installs the dependencies with `rosdep`;
2. builds every package;
3. runs the unit tests (`colcon test` + `colcon test-result --verbose`);
4. flies the x500 and the Bebop 2 together in the `empty` world with `flight_check.py`
   (Gazebo headless, `cameras:=false`);
5. flies the BlueROV2 Heavy in the `underwater` world (it needs water to float) with
   `flight_check.py`, including the floating and passive ascent checks;
6. flies a waypoint mission with an x500 and returns to launch with `mission_check.py`;
7. injects failures into an x500 with `failsafe_check.py`.

Each simulation step uses its own Gazebo partition and ROS domain and removes any Gazebo
server left at the end, so a step cannot interfere with the next one. Any failed check
fails the run, and the colcon and simulation logs are uploaded as the `logs` artifact. The same steps can be reproduced locally:

```bash
docker run --rm -it -v $PWD:/ws/src/pilotaro -w /ws ros:rolling bash
# inside the container, the "run" blocks of rolling.yml in order, e.g.:
apt-get update && rosdep update
rosdep install -y --ignore-src --from-paths src
source /opt/ros/rolling/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

[Back to the README](../README.md)
