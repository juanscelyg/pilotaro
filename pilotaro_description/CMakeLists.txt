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
project(pilotaro_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/update_vehicle_params.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Each vehicle: its URDF expands with xacro, has the motor plugins the script needs and
  # config/<vehicle>.params.yaml is in sync with it
  foreach(vehicle x500 bebop2 bluerov2)
    add_test(
      NAME ${vehicle}_vehicle_params_match_urdf
      COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/update_vehicle_params.py --check
        --urdf ${CMAKE_CURRENT_SOURCE_DIR}/urdf/${vehicle}.urdf.xacro
        --params ${CMAKE_CURRENT_SOURCE_DIR}/config/${vehicle}.params.yaml
    )
  endforeach()
endif()

ament_package()
