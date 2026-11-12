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

#ifndef PILOTARO_CORE__STATEESTIMATORBASE_HPP_
#define PILOTARO_CORE__STATEESTIMATORBASE_HPP_

#include "pilotaro_common/Types.hpp"
#include "pilotaro_core/PluginBase.hpp"

namespace pilotaro
{

/**
 * @class StateEstimatorBase
 * @brief Interface of the state estimator plugins, loaded by EstimatorNode.
 *
 * The plugin subscribes to whatever sensors it needs in on_initialize() (its callbacks
 * run in the cycle thread, before update()). On each cycle update() is called; when it
 * reports a new estimate, the rest of the system (flight modes, controller, actuators)
 * runs on it. The controller only runs on new estimates and takes its time step from the
 * difference of their stamps.
 */
class StateEstimatorBase : public PluginBase
{
public:
  /**
   * @brief Produce the newest estimate
   * @param[out] estimate state in world frame (ENU), angular velocity in body frame (FLU),
   * stamp in seconds
   * @return true if estimate holds a new estimate since the last call
   */
  virtual bool update(VehicleState & estimate) = 0;
};

}  // namespace pilotaro

#endif  // PILOTARO_CORE__STATEESTIMATORBASE_HPP_
