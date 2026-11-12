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

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "pilotaro_system/RealTime.hpp"

namespace pilotaro
{

std::string
set_real_time_priority(int priority)
{
  sched_param param {};
  param.sched_priority = priority;
  const int error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (error != 0) {
    rlimit limit {};
    getrlimit(RLIMIT_RTPRIO, &limit);
    return "SCHED_FIFO priority " + std::to_string(priority) + ": " + std::strerror(error) +
           " (RLIMIT_RTPRIO is " + std::to_string(limit.rlim_cur) +
           ", see 'ulimit -r' and /etc/security/limits.conf)";
  }
  return "";
}

std::string
set_cpu_affinity(int cpu)
{
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  const int error = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  if (error != 0) {
    return "CPU affinity " + std::to_string(cpu) + ": " + std::strerror(error);
  }
  return "";
}

std::string
lock_memory()
{
  rlimit limit {};
  if (getrlimit(RLIMIT_MEMLOCK, &limit) != 0) {
    return std::string("getrlimit(RLIMIT_MEMLOCK): ") + std::strerror(errno);
  }
  if (limit.rlim_cur != RLIM_INFINITY) {
    return "RLIMIT_MEMLOCK is " + std::to_string(limit.rlim_cur) +
           " bytes, it must be unlimited (see 'ulimit -l' and /etc/security/limits.conf)";
  }
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    return std::string("mlockall: ") + std::strerror(errno);
  }
  return "";
}

}  // namespace pilotaro
