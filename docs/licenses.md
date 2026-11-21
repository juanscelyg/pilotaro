# Licenses

pilotaro is licensed under the [Apache License 2.0](../LICENSE), Copyright 2026 Juan S. Cely G.
Every source, launch, URDF/xacro, world, parameter and build file carries that header.

Third-party content keeps its own license:

| Component | Where | Source | License |
|---|---|---|---|
| BlueROV2 Heavy meshes | [`pilotaro_description/meshes/bluerov2_heavy`](../pilotaro_description/meshes/bluerov2_heavy/README.md) (distributed, unmodified) | [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models) `uuv_bluerov2_heavy`, N. De Carli (KTH), PX4 Autopilot | BSD 3-Clause ([`LICENSE`](../pilotaro_description/meshes/bluerov2_heavy/LICENSE)) |
| x500 meshes | Downloaded from Fuel at run time, not distributed | [PX4 x500](https://app.gazebosim.org/PX4/fuel/models/x500), Benjamin Perseghetti | CC-BY 4.0 |
| Bebop 2 meshes | Downloaded from Fuel at run time, not distributed | [Parrot Bebop 2](https://app.gazebosim.org/OpenRobotics/fuel/models/Parrot%20Bebop%202), Open Robotics | CC0 |
| Simulation screenshots | [`docs/images`](images) | Gazebo renders of pilotaro | Apache 2.0; each vehicle shown keeps the license of its mesh (rows above) |

Physical parameters taken from other works (masses, inertias, thruster geometry, rotor and
hydrodynamic coefficients) are cited where they are used, in the header of each
`urdf/<vehicle>.urdf.xacro` and in [Vehicles](vehicles.md): the PX4 x500 model (CC-BY 4.0),
the Parrot Bebop 2 model of Open Robotics (CC0), S. Sun et al. (TU Delft, 2019) and the PX4
`uuv_bluerov2_heavy` model (PX4-gazebo-models, BSD 3-Clause).

[Back to the README](../README.md)
