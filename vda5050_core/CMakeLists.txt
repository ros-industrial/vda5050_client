cmake_minimum_required(VERSION 3.8)
project(vda5050_core VERSION 0.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
  # /bigobj: template-heavy deps (nlohmann_json, fmt, gmock) exceed MSVC's 64k
  #          section limit per object (fatal C1128).
  # /utf-8:  treat source/exec charset as UTF-8 instead of the system code page.
  add_compile_options(/bigobj /utf-8)
  # NOMINMAX: stop <windows.h> defining min/max macros that clash with
  #           std::min/max and std::numeric_limits<>::max().
  # _CRT_SECURE_NO_WARNINGS: silence C4996 "insecure" CRT deprecation warnings.
  add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
endif()

# Export all symbols from the shared libraries on Windows so consumers can link
# against the DLLs without per-symbol __declspec annotations. No-op elsewhere.
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Windows has no RPATH: co-locate the DLLs with the executables that load them
# (examples and tests) so they resolve at run time. Harmless on other platforms.
if(WIN32)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

find_package(PahoMqttCpp REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)

include(GNUInstallDirs)
include(cmake/vda5050_configure_target_alias.cmake)

# Types
add_library(vda5050_types INTERFACE)
vda5050_configure_target_alias(
  TARGET vda5050_types
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME types
)
target_include_directories(vda5050_types
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# JSON Utils
option(ENABLE_ROS2 "Enable ROS 2 message support in JSON utilities" OFF)
if(ENABLE_ROS2)
  message(STATUS "Building vda5050_json_utils with ROS 2 support")
  find_package(vda5050_interfaces REQUIRED)
else()
  message(STATUS "Building vda5050_json_utils without ROS 2 support")
endif()

add_library(vda5050_json_utils INTERFACE)
vda5050_configure_target_alias(
  TARGET vda5050_json_utils
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME json_utils
)
target_link_libraries(vda5050_json_utils
  INTERFACE
    ${PROJECT_NAME}::types
    nlohmann_json::nlohmann_json
)
target_include_directories(vda5050_json_utils
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(ENABLE_ROS2)
  target_link_libraries(vda5050_json_utils
    INTERFACE
      ${vda5050_interfaces_LIBRARIES}
  )
  target_include_directories(vda5050_json_utils
    INTERFACE
      ${vda5050_interfaces_INCLUDE_DIRS}
  )
  target_compile_definitions(vda5050_json_utils INTERFACE ENABLE_ROS2)
endif()

# Logger
add_library(vda5050_logger SHARED src/logger/logger.cpp)
vda5050_configure_target_alias(
  TARGET vda5050_logger
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME logger
)
target_link_libraries(vda5050_logger PUBLIC fmt::fmt)
target_include_directories(vda5050_logger
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Errors
add_library(vda5050_errors INTERFACE)
vda5050_configure_target_alias(
  TARGET vda5050_errors
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME errors
)
target_include_directories(vda5050_errors
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Order Utils
add_library(vda5050_order_utils SHARED src/order_utils/order_graph_validator.cpp)
vda5050_configure_target_alias(
  TARGET vda5050_order_utils
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME order_utils
)
target_link_libraries(vda5050_order_utils PUBLIC
  ${PROJECT_NAME}::types
  ${PROJECT_NAME}::errors
)
target_include_directories(vda5050_order_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Transport
add_library(vda5050_transport SHARED src/transport/paho_mqtt_client.cpp)
vda5050_configure_target_alias(
  TARGET vda5050_transport
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME transport
)
target_link_libraries(vda5050_transport
  PUBLIC
    PahoMqttCpp::paho-mqttpp3
    ${PROJECT_NAME}::logger
)
target_include_directories(vda5050_transport
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
# The shared paho-mqttpp3 DLL on Windows needs PAHO_MQTTPP_IMPORTS so its headers
# declare symbols as dllimport; the vcpkg target does not propagate it
# (microsoft/vcpkg#52261). PUBLIC because paho_mqtt_client.hpp exposes mqtt types.
if(WIN32)
  target_compile_definitions(vda5050_transport PUBLIC PAHO_MQTTPP_IMPORTS)
endif()

# Execution
file(GLOB_RECURSE execution_srcs "src/execution/*.cpp")
add_library(vda5050_execution SHARED ${execution_srcs})
vda5050_configure_target_alias(
  TARGET vda5050_execution
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME execution
)
target_link_libraries(vda5050_execution
  PUBLIC
    ${PROJECT_NAME}::types
    ${PROJECT_NAME}::json_utils
    ${PROJECT_NAME}::transport
)
target_include_directories(vda5050_execution
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Client
file(GLOB_RECURSE client_srcs "src/client/*.cpp")
add_library(vda5050_client SHARED ${client_srcs})
vda5050_configure_target_alias(
  TARGET vda5050_client
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME client
)
target_link_libraries(vda5050_client
  PUBLIC
    ${PROJECT_NAME}::execution
    ${PROJECT_NAME}::types
    ${PROJECT_NAME}::order_utils
    ${PROJECT_NAME}::logger
)
target_include_directories(vda5050_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Layout (LIF) — neutral namespace; master depends on it
file(GLOB_RECURSE layout_srcs "src/layout/*.cpp")
add_library(vda5050_layout SHARED ${layout_srcs})
vda5050_configure_target_alias(
  TARGET vda5050_layout
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME layout
)
target_link_libraries(vda5050_layout
  PUBLIC
    ${PROJECT_NAME}::types
    ${PROJECT_NAME}::json_utils
)
target_include_directories(vda5050_layout
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Master
file(GLOB_RECURSE master_srcs "src/master/*.cpp")
add_library(vda5050_master SHARED ${master_srcs})
vda5050_configure_target_alias(
  TARGET vda5050_master
  NAMESPACE ${PROJECT_NAME}::
  EXPORT_NAME master
)
target_link_libraries(vda5050_master
  PUBLIC
    ${PROJECT_NAME}::types
    ${PROJECT_NAME}::json_utils
    ${PROJECT_NAME}::transport
    ${PROJECT_NAME}::execution
)
target_include_directories(vda5050_master
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(
  TARGETS
    vda5050_types
    vda5050_json_utils
    vda5050_logger
    vda5050_errors
    vda5050_order_utils
    vda5050_transport
    vda5050_execution
    vda5050_client
    vda5050_layout
    vda5050_master
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Export the targets and generate a plain CMake package config so consumers can
# `find_package(vda5050_core)` and link `vda5050_core::<component>`.
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  cmake/vda5050_ament_lint_common.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

option(BUILD_EXAMPLES "Build examples" ON)

if(BUILD_EXAMPLES)
  add_executable(custom_base examples/custom_base.cpp)
  target_link_libraries(custom_base PUBLIC
    ${PROJECT_NAME}::execution
    ${PROJECT_NAME}::logger
  )

  add_executable(provider_example examples/provider_example.cpp)
  target_link_libraries(provider_example PUBLIC
    ${PROJECT_NAME}::execution
    ${PROJECT_NAME}::logger
  )

  add_executable(engine_example examples/engine_example.cpp)
  target_link_libraries(engine_example PUBLIC
    ${PROJECT_NAME}::execution
    ${PROJECT_NAME}::logger
  )

  add_executable(handler_integration examples/handler_integration.cpp)
  target_link_libraries(handler_integration PUBLIC
    ${PROJECT_NAME}::execution
    ${PROJECT_NAME}::logger
  )

  install(TARGETS
      custom_base
      provider_example
      engine_example
      handler_integration
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  )
endif()

option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
  enable_testing()

  # In a ROS 2 dev environment, keep the ament linters and register tests with
  # the ament backends (selected inside vda5050_add_gtest/gmock by the
  # ament_cmake_g{test,mock}_FOUND variables set here).
  find_package(ament_cmake_gtest QUIET)
  find_package(ament_cmake_gmock QUIET)
  if(ament_cmake_gmock_FOUND)
    include(cmake/vda5050_ament_lint_common.cmake)
    vda5050_ament_lint_common()
  endif()

  include(cmake/vda5050_add_gtest.cmake)
  include(cmake/vda5050_add_gmock.cmake)

  set(json_tests test/json_utils/test_json_utils.cpp)
  if(ENABLE_ROS2)
    list(APPEND json_tests test/json_utils/test_json_utils_ros2.cpp)
  endif()
  vda5050_add_gtest(test_json_utils ${json_tests})
  target_link_libraries(test_json_utils vda5050_core::json_utils)

  vda5050_add_gtest(test_order_utils
    test/order_utils/test_unit_order_graph_validator.cpp)
  target_link_libraries(test_order_utils vda5050_core::order_utils)

  vda5050_add_gtest(test_logger
    test/logger/test_unit_default_logger.cpp
    test/logger/test_unit_logger.cpp
  )
  target_link_libraries(test_logger vda5050_core::logger)

  vda5050_add_gmock(test_transport
    test/transport/test_unit_mqtt_client_interface.cpp
    test/transport/test_integration_paho_mqtt_client.cpp
  )
  target_link_libraries(test_transport vda5050_core::transport)

  vda5050_add_gmock(test_client
    test/client/test_order_types.cpp
    test/client/test_order_execution_resource.cpp
    test/client/test_agv_context.cpp
    test/client/test_order_validator.cpp
    test/client/test_order_acceptance.cpp
  )
  target_link_libraries(test_client vda5050_core::client)

  vda5050_add_gmock(test_execution
    test/execution/test_engine.cpp
    test/execution/test_event_queue.cpp
    test/execution/test_handler.cpp
    test/execution/test_protocol_adapter.cpp
    test/execution/test_provider.cpp
  )
  target_link_libraries(test_execution vda5050_core::execution)

  vda5050_add_gmock(test_master
    test/master/test_integration_heartbeat.cpp
    test/master/test_integration_master_logic.cpp
    test/master/test_integration_master_mqtt.cpp
    test/master/test_integration_master_teardown.cpp
    test/master/test_unit_agv.cpp
  )
  target_link_libraries(test_master vda5050_core::master)

  vda5050_add_gtest(test_layout_loader
    test/layout/test_unit_loader.cpp
  )
  target_link_libraries(test_layout_loader vda5050_core::layout)
  target_compile_definitions(test_layout_loader PRIVATE
    VDA5050_CORE__SAMPLE_LAYOUT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/layout/data/sample_layout.json")

  vda5050_add_gtest(test_layout_graph
    test/layout/test_unit_graph.cpp
  )
  target_link_libraries(test_layout_graph vda5050_core::layout)
  target_compile_definitions(test_layout_graph PRIVATE
    VDA5050_CORE__SAMPLE_LAYOUT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/layout/data/sample_layout.json")
endif()
