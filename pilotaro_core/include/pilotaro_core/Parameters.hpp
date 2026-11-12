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

#ifndef PILOTARO_CORE__PARAMETERS_HPP_
#define PILOTARO_CORE__PARAMETERS_HPP_

#include <Eigen/Dense>

#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter_value.hpp"

#include "pilotaro_common/Types.hpp"

namespace pilotaro
{

using ParametersInterface = rclcpp::node_interfaces::NodeParametersInterface::SharedPtr;

/**
 * @brief Declare a parameter, or get it if it was already declared (e.g. on a second
 * configure after a cleanup).
 * @param params parameters interface of the node
 * @param name full parameter name
 * @param default_value value used when there is no override
 * @return current value
 */
template<typename T>
T declare_or_get(
  const ParametersInterface & params, const std::string & name,
  const T & default_value)
{
  if (!params->has_parameter(name)) {
    return params->declare_parameter(name, rclcpp::ParameterValue(default_value))
           .template get<T>();
  }
  return params->get_parameter(name).get_value<T>();
}

/**
 * @brief Declare a 3 element vector parameter
 * @throws std::invalid_argument if it does not have 3 elements
 */
inline Eigen::Vector3d declare_vector3(
  const ParametersInterface & params, const std::string & name,
  const Eigen::Vector3d & default_value)
{
  const auto v = declare_or_get<std::vector<double>>(
    params, name, {default_value.x(), default_value.y(), default_value.z()});
  if (v.size() != 3) {
    throw std::invalid_argument("Parameter " + name + " must have 3 elements");
  }
  return {v[0], v[1], v[2]};
}

/**
 * @brief Declare the vehicle parameters ("<prefix>.mass", "<prefix>.inertia"...)
 * @param params parameters interface of the node
 * @param prefix parameter prefix
 * @return parameters, not validated
 */
inline VehicleParams declare_vehicle_params(
  const ParametersInterface & params, const std::string & prefix = "vehicle")
{
  VehicleParams v;
  const std::string p = prefix + ".";
  v.mass = declare_or_get(params, p + "mass", v.mass);
  v.gravity = declare_or_get(params, p + "gravity", v.gravity);
  v.inertia = declare_vector3(params, p + "inertia", v.inertia);
  v.rotor_x = declare_or_get(params, p + "rotor_x", v.rotor_x);
  v.rotor_y = declare_or_get(params, p + "rotor_y", v.rotor_y);
  v.rotor_z = declare_or_get(params, p + "rotor_z", v.rotor_z);
  v.rotor_axis = declare_or_get(params, p + "rotor_axis", v.rotor_axis);
  v.rotor_direction = declare_or_get(params, p + "rotor_direction", v.rotor_direction);
  v.motor_constant = declare_or_get(params, p + "motor_constant", v.motor_constant);
  v.moment_constant = declare_or_get(params, p + "moment_constant", v.moment_constant);
  v.min_rot_velocity = declare_or_get(params, p + "min_rot_velocity", v.min_rot_velocity);
  v.max_rot_velocity = declare_or_get(params, p + "max_rot_velocity", v.max_rot_velocity);
  v.reversible = declare_or_get(params, p + "reversible", v.reversible);
  return v;
}

}  // namespace pilotaro

#endif  // PILOTARO_CORE__PARAMETERS_HPP_
