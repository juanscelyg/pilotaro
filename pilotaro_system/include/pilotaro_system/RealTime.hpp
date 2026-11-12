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

#ifndef PILOTARO_SYSTEM__REALTIME_HPP_
#define PILOTARO_SYSTEM__REALTIME_HPP_

#include <string>

namespace pilotaro
{

/**
 * @brief Run the calling thread with SCHED_FIFO
 * @param priority 1 (lowest) to 99 (highest), limited by RLIMIT_RTPRIO
 * @return empty on success, the reason otherwise
 */
std::string set_real_time_priority(int priority);

/**
 * @brief Pin the calling thread to one CPU
 * @param cpu CPU index
 * @return empty on success, the reason otherwise
 */
std::string set_cpu_affinity(int cpu);

/**
 * @brief Lock the current and future memory of the process in RAM (no page faults)
 * @return empty on success, the reason otherwise
 */
std::string lock_memory();

}  // namespace pilotaro

#endif  // PILOTARO_SYSTEM__REALTIME_HPP_
