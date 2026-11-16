# BlueROV2 Heavy meshes

`BlueROV2heavy.dae` (hull) and `prop.dae` (thruster propeller) are the visuals of
`urdf/bluerov2_heavy.urdf.xacro`. They are used only for display; the collision and the
buoyancy come from a box (see the URDF).

- Source: [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models),
  `models/uuv_bluerov2_heavy/meshes`, commit `3eb05f716a81bca316ab5771f53e509a07dce3a6`.
  Model by Nicola De Carli (KTH).
- License: BSD 3-Clause, Copyright (c) 2022, PX4 Autopilot for Drones
  ([`LICENSE`](LICENSE)).
- Changes: none. The hull visual keeps the pose of the PX4 model (0.06 m below
  `base_link`).
