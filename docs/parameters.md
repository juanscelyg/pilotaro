# Parameters

Each instance loads two files, in this order:

1. `pilotaro/params/pilotaro.params.yaml` (`params_file`): what is common to all vehicles,
   i.e. the loop, real time, plugin selection and configuration, flight modes and actuators.
2. `pilotaro_description/config/<vehicle>.params.yaml` (`vehicle` or `vehicle_params_file`):
   the `vehicle` block (mass, inertia, rotors, motors), generated from the URDF, and the
   vehicle's own tuning. For example, the Bebop 2 changes the controller's velocity gains.
   Since it is loaded second, it can override any parameter of the first one.

The parameter files use `/**/<node>:`, so they work for every robot of a multi-robot
system; a robot that needs its own values gets a different `params_file` or
`vehicle_params_file` ([Quick start](quickstart.md#multi-robot-platform-name)).

What each block configures: [Architecture and plugins](architecture.md) (plugin selection),
[State estimators](estimators.md), [Controllers](controllers.md), [Flight modes](modes.md)
and [Failsafes](failsafes.md) (`states_node`), [Real time](realtime.md) (`pilotaro_node`) and
[Vehicles](vehicles.md) (the `vehicle` block, generated from the URDF).

[Back to the README](../README.md)
