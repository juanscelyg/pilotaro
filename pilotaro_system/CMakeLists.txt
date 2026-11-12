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
project(pilotaro_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pilotaro_common REQUIRED)
find_package(pilotaro_core REQUIRED)
find_package(pilotaro_states REQUIRED)
find_package(pilotaro_estimators REQUIRED)
find_package(pilotaro_actuators REQUIRED)
find_package(pilotaro_controllers REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/pilotaro_system/PilotaroNode.cpp
  src/pilotaro_system/RealTime.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pilotaro_common::pilotaro_common
  pilotaro_core::pilotaro_core
  pilotaro_states::pilotaro_states
  pilotaro_estimators::pilotaro_estimators
  pilotaro_actuators::pilotaro_actuators
  pilotaro_controllers::pilotaro_controllers
  ${lifecycle_msgs_TARGETS}
)

add_executable(pilotaro_main src/pilotaro_main.cpp)
target_link_libraries(pilotaro_main PUBLIC
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  pilotaro_main
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  pilotaro_common
  pilotaro_core
  pilotaro_states
  pilotaro_estimators
  pilotaro_actuators
  pilotaro_controllers
)
ament_package()
