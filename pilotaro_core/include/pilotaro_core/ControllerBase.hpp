//  Copyright 2026 Juan S. Cely G.

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      https://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef PILOTARO_CORE__CONTROLLERBASE_HPP_
#define PILOTARO_CORE__CONTROLLERBASE_HPP_

#include <vector>

#include "pilotaro_common/Types.hpp"
#include "pilotaro_core/PluginBase.hpp"

namespace pilotaro
{

/**
 * @class ControllerBase
 * @brief Interface of the controller plugins, loaded by ControllersNode.
 *
 * Called only while flying (TAKEOFF, LOITER, LANDING, OFFBOARD) and once per new state
 * estimate. reset() is called when the vehicle starts flying from the ground.
 */
class ControllerBase : public PluginBase
{
public:
  /**
   * @brief Compute the rotor speeds for one control step
   * @param state current state estimate
   * @param setpoint desired reference
   * @param dt time since the previous step [s]
   * @param[out] rotor_speeds angular velocity of each rotor [rad/s], already sized to
   * the number of rotors
   * @return false if no valid command could be computed
   */
  virtual bool update(
    const VehicleState & state, const ControlSetpoint & setpoint, double dt,
    std::vector<double> & rotor_speeds) = 0;
};

}  // namespace pilotaro

#endif  // PILOTARO_CORE__CONTROLLERBASE_HPP_
