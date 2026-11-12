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

#ifndef PILOTARO_CORE__ACTUATOROUTPUTBASE_HPP_
#define PILOTARO_CORE__ACTUATOROUTPUTBASE_HPP_

#include <vector>

#include "pilotaro_core/PluginBase.hpp"

namespace pilotaro
{

/**
 * @class ActuatorOutputBase
 * @brief Interface of the actuator output plugins, loaded by ActuatorsNode.
 *
 * ActuatorsNode decides the rotor speeds of every cycle (zero when DISARMED, idle when
 * ARMED, the controller output while flying) and the plugin sends them to the hardware or
 * the simulator in its own format: one message for every motor, one topic per thruster, a
 * serial protocol...
 */
class ActuatorOutputBase : public PluginBase
{
public:
  /**
   * @brief Send the rotor speeds to the actuators. Called once per cycle.
   * @param rotor_speeds angular velocity of each rotor [rad/s], negative for reversed
   * thrusters
   * @return false if they could not be sent
   */
  virtual bool write(const std::vector<double> & rotor_speeds) = 0;
};

}  // namespace pilotaro

#endif  // PILOTARO_CORE__ACTUATOROUTPUTBASE_HPP_
