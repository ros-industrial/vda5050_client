# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Dual-mode gmock test registration for VDA5050.
#
# Drop-in replacement for ament_add_gmock(<target> <sources...>): link the test
# against its libraries with a normal target_link_libraries() call afterwards.
#
# When ament_cmake_gmock is available (the caller is expected to have run
# find_package(ament_cmake_gmock) so ament_cmake_gmock_FOUND is set), tests
# register via ament_add_gmock for `colcon test`. Otherwise they fall back to
# upstream GoogleTest (gmock + gtest + main via GTest::gmock_main).
#
# Each executable is registered as a single ctest entry (matching the ament
# behaviour), so the gtest cases within it run serially -- important for the
# integration tests that share a single MQTT broker.
#
# vda5050_add_gmock(<target> <source> [<source> ...])
#
function(vda5050_add_gmock target)
  if(ament_cmake_gmock_FOUND)
    ament_add_gmock(${target} ${ARGN})
  else()
    # Find GoogleTest on first use.
    if(NOT GTest_FOUND)
      find_package(GTest REQUIRED)
    endif()
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} GTest::gmock_main)
    add_test(NAME ${target} COMMAND ${target})
  endif()
endfunction()
