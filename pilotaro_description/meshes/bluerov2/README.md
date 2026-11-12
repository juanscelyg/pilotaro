# BlueROV2 visual mesh

`bluerov2.glb` is the visual of `urdf/bluerov2.urdf.xacro`. It is used only for display;
the collisions, mass and buoyancy of the vehicle are defined in the URDF.

## Origin

`model/BlueRov2.dae` from [bluerov_ros_playground](https://github.com/patrickelectric/bluerov_ros_playground)
(Patrick José Pereira, Blue Robotics, commit `75c6adb`, 2018), licensed under the
GNU LGPL v3 (`package.xml` of that repository).

## Changes

- Geometry grouped by material: one material per diffuse color of the original (27).
- Rotated to the pilotaro `base_link` frame (x forward, z up). The original has y forward
  and z down: roll π, then yaw π/2.
- Translated by (0.0088, 0, 0.0096) m: centered in x and the skids at z = −0.11 m, so that
  the T200 thrusters match the thruster joints of the URDF.
- Vertices welded (shared between triangles), with all the detail of the original:
  512 618 triangles, exported as binary glTF with z up (Gazebo reads it as is; no y-up
  conversion). 20 MB instead of the 44 MB of the original DAE.

Its cost in simulation is small (RTX 4070, headless, with the front camera): real-time
factor 1.0 and camera at 29 Hz with one ROV, and 1.0 and 26 Hz with three, the same as a
version decimated to 49k triangles. The server uses about 150 MB more memory.

## License

This mesh is a modified version of a work under the GNU Lesser General Public License v3
(`COPYING.LESSER`, which supplements the GNU GPL v3 in `COPYING`). It keeps that license.
The rest of pilotaro is under the Apache License 2.0 and only refers to this file by path.
