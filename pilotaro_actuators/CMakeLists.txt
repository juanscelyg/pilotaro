# Copyright 2026 Juan S. Cely G.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)
project(pilotaro_actuators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pilotaro_common REQUIRED)
find_package(pilotaro_core REQUIRED)

# Node that applies the output policy and hosts the output plugin
add_library(${PROJECT_NAME} SHARED
  src/pilotaro_actuators/ActuatorsNode.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  pluginlib::pluginlib
  pilotaro_common::pilotaro_common
  pilotaro_core::pilotaro_core
)

# Output plugins
add_library(${PROJECT_NAME}_plugins SHARED
  src/pilotaro_actuators/OutputPlugins.cpp
)
target_include_directories(${PROJECT_NAME}_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC
  rclcpp::rclcpp
  pluginlib::pluginlib
  ${actuator_msgs_TARGETS}
  ${std_msgs_TARGETS}
  pilotaro_core::pilotaro_core
)
pluginlib_export_plugin_description_file(pilotaro_core output_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_actuators test/test_actuators.cpp)
  target_link_libraries(test_actuators ${PROJECT_NAME} ${PROJECT_NAME}_plugins)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_plugins)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  pluginlib
  actuator_msgs
  std_msgs
  pilotaro_common
  pilotaro_core
)
ament_package()
