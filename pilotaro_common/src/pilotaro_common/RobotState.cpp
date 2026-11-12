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

#include <iostream>

#include "pilotaro_common/RobotState.hpp"

namespace pilotaro
{

void
RobotState::set_estimate(const VehicleState & estimate, double reception_time)
{
  estimate_ = estimate;
  estimate_reception_ = reception_time;
  ++estimate_seq_;
}

bool
RobotState::consume_controller_reset()
{
  const bool requested = controller_reset_;
  controller_reset_ = false;
  return requested;
}

void
RobotState::show() const
{
  std::cout << "------ :: Pilotaro State :: ------" << std::endl;
  std::cout << " \t -- name: " << get_name() << std::endl;
  std::cout << " \t -- available: " << (is_available() ? "On" : "Off") << std::endl;
  std::cout << " \t -- simulation: " << (is_simulation() ? "On" : "Off") << std::endl;
  std::cout << " \t -- mode: " << to_string(get_mode()) << std::endl;
  std::cout << " \t -- vehicle: " << vehicle_.rotor_count() << " rotors, " << vehicle_.mass <<
    " kg, hover at " << vehicle_.hover_rot_velocity() << " rad/s" << std::endl;
  std::cout << "------------------------------------" << std::endl;
}

}  // namespace pilotaro
