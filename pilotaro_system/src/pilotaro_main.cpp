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

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pilotaro_system/PilotaroNode.hpp"
#include "pilotaro_system/RealTime.hpp"

namespace
{

/**
 * @brief Apply the real_time.* parameters to the calling thread and the process
 * @return false if real time is required and could not be set
 */
bool setup_real_time(const pilotaro::PilotaroNode::SharedPtr & node)
{
  if (!node->get_parameter("real_time.enabled").as_bool()) {
    RCLCPP_INFO(node->get_logger(), "Real time disabled (real_time.enabled = false)");
    return true;
  }
  const bool required = node->get_parameter("real_time.required").as_bool();
  const auto report = [&](const std::string & what, const std::string & error) {
      if (error.empty()) {
        RCLCPP_INFO(node->get_logger(), "[real_time] %s", what.c_str());
        return true;
      }
      if (required) {
        RCLCPP_FATAL(node->get_logger(), "[real_time] %s: %s", what.c_str(), error.c_str());
        return false;
      }
      RCLCPP_WARN(
        node->get_logger(), "[real_time] %s failed, running without it: %s", what.c_str(),
        error.c_str());
      return true;
    };

  bool ok = true;
  if (node->get_parameter("real_time.lock_memory").as_bool()) {
    ok = report("Memory locked", pilotaro::lock_memory()) && ok;
  }
  const int cpu = static_cast<int>(node->get_parameter("real_time.cpu").as_int());
  if (cpu >= 0) {
    ok = report("Pinned to CPU " + std::to_string(cpu), pilotaro::set_cpu_affinity(cpu)) && ok;
  }
  const int priority = static_cast<int>(node->get_parameter("real_time.priority").as_int());
  ok = report(
    "SCHED_FIFO priority " + std::to_string(priority),
    pilotaro::set_real_time_priority(priority)) && ok;
  return ok;
}

}  // namespace

int main(int argc, char * argv[])
{
  std::atomic_bool working{true};

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto pilotaro_node = pilotaro::PilotaroNode::make_shared();

  exe.add_node(pilotaro_node->get_node_base_interface());
  for (auto & pilotaro_system_node : pilotaro_node->get_pilotaro_system_nodes()) {
    exe.add_node(pilotaro_system_node.second.node_ptr);
  }

  const double frequency = pilotaro_node->get_parameter("frequency").as_double();
  const double spin_time = pilotaro_node->get_parameter("spin_time").as_double();
  if (frequency <= 0.0 || spin_time <= 0.0 || spin_time >= 1.0 / frequency) {
    RCLCPP_FATAL(
      pilotaro_node->get_logger(), "frequency must be > 0 and 0 < spin_time < 1 / frequency");
    rclcpp::shutdown();
    return 1;
  }
  const auto spin_duration =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(spin_time));

  pilotaro_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (pilotaro_node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(pilotaro_node->get_logger(), "Unable to configure Pilotaro");
    rclcpp::shutdown();
    return 1;
  }

  // After configure (plugins loaded, memory allocated) and before the vehicle can move
  if (!setup_real_time(pilotaro_node)) {
    rclcpp::shutdown();
    return 1;
  }

  pilotaro_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (pilotaro_node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(pilotaro_node->get_logger(), "Unable to activate Pilotaro");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::on_shutdown([&](){
      working.store(false, std::memory_order_relaxed);
      exe.cancel();
  });

  // One thread does everything: take the incoming messages and service calls, then run
  // estimator -> states -> controllers -> actuators on them
  const auto period = std::chrono::duration<double>(1.0 / frequency);
  rclcpp::WallRate rate(frequency);
  auto last_report = std::chrono::steady_clock::now();
  uint64_t overruns = 0, cycles = 0;
  while (working.load(std::memory_order_relaxed)) {
    const auto start = std::chrono::steady_clock::now();
    exe.spin_all(spin_duration);

    if (pilotaro_node->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      pilotaro_node->pilotaro_cycle();
    }

    const auto end = std::chrono::steady_clock::now();
    ++cycles;
    if (end - start > period) {
      ++overruns;
    }
    if (end - last_report > std::chrono::seconds(10)) {
      if (overruns > 0) {
        RCLCPP_WARN(
          pilotaro_node->get_logger(), "%lu of %lu cycles took longer than %.2f ms",
          static_cast<unsigned long>(overruns), static_cast<unsigned long>(cycles),  // NOLINT
          1000.0 * period.count());
      }
      overruns = 0;
      cycles = 0;
      last_report = end;
    }
    rate.sleep();
  }

  working.store(false, std::memory_order_relaxed);
  exe.cancel();

  rclcpp::shutdown();
  return 0;
}
