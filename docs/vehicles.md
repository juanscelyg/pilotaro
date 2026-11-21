# Vehicles

Each vehicle in `pilotaro_description` has its URDF in `urdf/<vehicle>.urdf.xacro`, built
with the macros in [`urdf/common.xacro`](../pilotaro_description/urdf/common.xacro) (rotor with
its `MulticopterMotorModel`, marine thruster with its `Thruster`, hydrodynamics, IMU,
barometer, magnetometer, GPS, camera, rangefinder and odometry), and its parameters in
`config/<vehicle>.params.yaml`. All topics are prefixed with the robot name.

| | x500 | bebop2 | bluerov2_heavy |
|---|---|---|---|
| Model | Holybro X500 (PX4) | Parrot Bebop 2 | Blue Robotics BlueROV2 Heavy (8 thrusters, PX4 model) |
| Mass | 2.114 kg | 0.503 kg | 13.4 kg, buoyancy +0.98 N |
| Actuators (order) | 0 FR ccw, 1 RL ccw, 2 FL cw, 3 RR cw (PX4) | 0 FR ccw, 1 FL cw, 2 RL ccw, 3 RR cw (Parrot) | 4 horizontal at 45° (0 FR, 1 FL, 2 RR, 3 RL) and 4 vertical at the corners (4 FR up, 5 FL down, 6 RR down, 7 RL up) (PX4) |
| `motor_constant` | 8.55e-6 N/(rad/s)² | 1.90e-6 N/(rad/s)² | 0.002 N/(rad/s)², ±49.7 N |
| Hover / max | 778 / 1000 rad/s (T/W 1.65) | 805 / 1250 rad/s (T/W 2.4) | reversible, ±158 rad/s |
| Controller / output | Cascade / MotorSpeed | Cascade / MotorSpeed | Underwater / Thruster |
| IMU, magnetometer | 250, 50 Hz; IMU noise of the PX4 x500 model | 250, 50 Hz | 250, 50 Hz; IMU noise of the PX4 BlueROV2 Heavy model |
| Barometer, GPS | 50, 10 Hz | 50, 10 Hz | |
| Front camera `camera_front` | 1280×960, 97°, 30 Hz | 1280×720, 80°, 30 Hz | 1280×720, 80°, 30 Hz |
| Downward camera `camera_down` | 640×360, 86°, 30 Hz | 320×240 (optical flow), 64°, 30 Hz | |
| Range to ground `sonar` (`LaserScan`) | | ultrasound, 0.25–8 m, 30 Hz | Ping echosounder, 0.3–30 m, 10 Hz |
| Thrusters (input) | | | `thruster_<i>/cmd_thrust` (`Float64`, N) |
| Spawn | at rest, 0.24 m | at rest, 0.065 m | at the surface, z = 0 |

(FR: front right, FL: front left, RL: rear left, RR: rear right.)

Data sources:

- **x500:** PX4 model on Fuel (geometry, masses, motors and meshes, CC-BY 4.0).
- **bebop2:** mass, inertia tensor, propeller positions and meshes from Open Robotics'
  "Parrot Bebop 2" model on Fuel (CC0). Rotor thrust, torque and drag coefficients from the
  aerodynamic model identified in flight by S. Sun et al. (TU Delft, 2019,
  [bebop2_aerodynamic_model](https://bitbucket.org/SihaoSun/bebop2_aerodynamic_model)).
  Parrot rotor order and spin directions, as in BebopS. Details are in the header of
  [`bebop2.urdf.xacro`](../pilotaro_description/urdf/bebop2.urdf.xacro).
- **bluerov2_heavy:** the same vehicle as PX4's `uuv_bluerov2_heavy` model
  ([PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models), BSD 3-Clause; airframe
  `60002_gz_uuv_bluerov2_heavy`), so that pilotaro and PX4 can be compared on one platform:
  - Rigid body: 13 kg hull with inertia (0.250, 0.221, 0.356) kg·m² plus 8 thrusters of
    0.05 kg. The thruster links here weigh 2 g, so the hull is set for the same totals:
    13.4 kg and (0.263, 0.229, 0.375) kg·m² about the center of mass.
  - Thruster positions, orientations and spin directions of the PX4 model.
  - Added mass, linear and quadratic damping of the PX4 model.
  - Buoyancy: its 0.50 × 0.25 × 0.108 m box, 2 cm above `base_link`: +0.98 N net.
  - Thrust limit: the PX4 thrust polynomials give +66.5 / −49.7 N at full command; the URDF
    limits both directions to 49.7 N, so the allocation stays symmetric.
  - Visual: the PX4 meshes, in
    [`meshes/bluerov2_heavy`](../pilotaro_description/meshes/bluerov2_heavy/README.md), which
    keep their BSD 3-Clause license; the rest of pilotaro stays Apache 2.0.

The x500 and Bebop 2 meshes are downloaded from Fuel the first time (an internet connection
is needed); the BlueROV2 Heavy meshes are installed with `pilotaro_description`. With
`cameras:=false` the models have no cameras, which makes multi-drone simulations much
lighter.

## Underwater vehicles and the `underwater` world

[`worlds/underwater.sdf`](../pilotaro_gazebo/worlds/underwater.sdf) has water (1000 kg/m³)
below z = 0 and air above, with the `Buoyancy` system in `graded_buoyancy` mode: a partially
submerged body gets the buoyancy of its submerged part, so a vehicle with positive buoyancy
really floats at the surface. The sandy bottom is at 10 m, with a pipeline and two rocks to
look at with the camera and the echosounder.

In the URDF, each underwater vehicle has:

- **Buoyancy:** the volume of its `base_link` collisions times the water density. On the
  BlueROV2 Heavy, the box gives the weight plus 0.98 N, with its center 2 cm above the
  center of mass, which stabilizes roll and pitch.
- **Hydrodynamics:** `Hydrodynamics` system with linear and quadratic damping,
  `pilotaro_hydrodynamics` macro.
- **Added mass:** the plugin's explicit computation is unstable when it exceeds the link
  mass, as it does in sway and heave on light ROV hulls. So it is
  disabled in the plugin (`disable_added_mass`) and `spawn.launch.py` passes it to the
  link's `<fluid_added_mass>` when generating the SDF. There DART integrates it implicitly
  and stably.
- **Thrusters:** `Thruster` system in thrust mode, `pilotaro_thruster` macro.

For an underwater vehicle:

- **"Takeoff"** with negative `takeoff_altitude`: dives that depth from the arming point
  (2 m on the BlueROV2 Heavy).
- **"Landing":** descends at `landing_speed` until it rests on the bottom, then disarms.
- **Surfacing:** with offboard or, with positive buoyancy, by disarming.

## Parameters from the URDF

The URDF is the single source of truth for the vehicle. If it changes (mass, arms, motors,
sensors), the `vehicle` block of its parameter file must be regenerated. There is one test
per vehicle (`colcon test --packages-select pilotaro_description`) that fails if they do not
match.

```bash
# From src/pilotaro/pilotaro_description/scripts
./update_vehicle_params.py                    # x500: urdf/x500.urdf.xacro -> config/x500.params.yaml
./update_vehicle_params.py --vehicle bebop2   # bebop2
./update_vehicle_params.py --dry-run          # only prints the block
./update_vehicle_params.py --check            # exits with 1 if the file does not match the URDF
./update_vehicle_params.py --xacro-arg cameras:=false

# From anywhere, with explicit paths
ros2 run pilotaro_description update_vehicle_params.py \
  --urdf src/pilotaro/pilotaro_description/urdf/x500.urdf.xacro \
  --params src/pilotaro/pilotaro_description/config/x500.params.yaml
```

The script computes, using all the links in the URDF:

- **mass:** the sum over all links;
- **center of mass:** that of the whole vehicle;
- **inertia:** the diagonal of the inertia tensor about the center of mass, using Steiner;
- **rotor positions:** relative to the center of mass.

Motor constants and spin directions are read from the `MulticopterMotorModel` plugins. It
only changes the lines of the `vehicle` block and keeps `gravity` and `min_rot_velocity`,
which are not in the URDF.

No vehicle is symmetric about its center of mass:

- **x500:** the cameras move the center of mass 2 mm forward, so the `rotor_x` values are
  0.1719 / -0.1761.
- **Bebop 2:** the front propellers are at 0.085 m and the rear ones at 0.090 m.

In both cases the front rotors carry slightly more thrust in hover. The controller's
allocation accounts for it.

## Adding a vehicle

1. Write `urdf/<vehicle>.urdf.xacro` with the macros in `common.xacro`: one
   `pilotaro_rotor` per motor (multirotor) or one `pilotaro_thruster` per thruster, with
   `pilotaro_hydrodynamics` (underwater), and whatever sensors it has. See
   `bebop2.urdf.xacro` and `bluerov2_heavy.urdf.xacro` as examples.
2. Generate its parameters: `./update_vehicle_params.py --vehicle <vehicle>`. The file is
   created if it does not exist.
3. Add its resting height to `SPAWN_HEIGHT` in `pilotaro_gazebo/launch/spawn.launch.py` and
   the vehicle to the test list in `pilotaro_description/CMakeLists.txt`.
4. Test it with `flight_check.py` (see [Testing](testing.md)) and, if needed, tune the controller
   in `config/<vehicle>.params.yaml`.

[Back to the README](../README.md)
