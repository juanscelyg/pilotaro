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

#ifndef PILOTARO_CORE__PLUGINBASE_HPP_
#define PILOTARO_CORE__PLUGINBASE_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "pilotaro_common/Types.hpp"
#include "pilotaro_core/Parameters.hpp"

namespace pilotaro
{

/**
 * @class PluginBase
 * @brief Common part of the pilotaro plugins (estimators and controllers).
 *
 * A plugin is created by pluginlib from the "<plugin_name>.plugin" parameter of its host
 * node, and reads its own parameters under "<plugin_name>." in the same node, so all of
 * them live in the main params file. Every method is called from the pilotaro cycle
 * thread, which may run with real-time priority: avoid blocking calls and allocations
 * in the update methods.
 */
class PluginBase
{
public:
  virtual ~PluginBase() = default;

  /**
   * @brief Store the host node and configure the plugin
   * @param node host node; the plugin keeps a weak reference
   * @param plugin_name name used as parameter prefix and logger name
   * @param vehicle physical properties of the vehicle
   * @return result of on_initialize()
   */
  bool initialize(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const VehicleParams & vehicle)
  {
    node_ = node;
    plugin_name_ = plugin_name;
    vehicle_ = vehicle;
    return on_initialize();
  }

  /**
   * @brief Reset the internal state (filters, integrators)
   */
  virtual void reset() {}

  /**
   * @brief Name used as parameter prefix
   */
  const std::string & get_plugin_name() const {return plugin_name_;}

protected:
  /**
   * @brief Read parameters and create subscriptions. Called once by initialize().
   * @return false if the plugin cannot work with this configuration
   */
  virtual bool on_initialize() = 0;

  /**
   * @brief Host node
   * @throws std::runtime_error if the node was destroyed
   */
  rclcpp::Node::SharedPtr get_node() const
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("pilotaro plugin " + plugin_name_ + " used without a node");
    }
    return node;
  }

  /**
   * @brief Physical properties of the vehicle
   */
  const VehicleParams & get_vehicle() const {return vehicle_;}

  /**
   * @brief Logger of the plugin, child of the node logger
   */
  rclcpp::Logger get_logger() const {return get_node()->get_logger().get_child(plugin_name_);}

  /**
   * @brief Declare a parameter of the plugin ("<plugin_name>.<name>")
   * @param name parameter name without the prefix
   * @param default_value value used when the params file does not set it
   * @return current value
   */
  template<typename T>
  T declare_parameter(const std::string & name, const T & default_value)
  {
    return declare_or_get(
      get_node()->get_node_parameters_interface(), plugin_name_ + "." + name, default_value);
  }

  /**
   * @brief Declare a 3 element vector parameter of the plugin
   */
  Eigen::Vector3d declare_vector3(const std::string & name, const Eigen::Vector3d & default_value)
  {
    return pilotaro::declare_vector3(
      get_node()->get_node_parameters_interface(), plugin_name_ + "." + name, default_value);
  }

private:
  std::weak_ptr<rclcpp::Node> node_;       ///< Host node
  std::string plugin_name_;       ///< Parameter prefix
  VehicleParams vehicle_;       ///< Physical properties
};

}  // namespace pilotaro

#endif  // PILOTARO_CORE__PLUGINBASE_HPP_
