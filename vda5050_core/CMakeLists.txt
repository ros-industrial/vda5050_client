cmake_minimum_required(VERSION 3.8)
project(vda5050_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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
file(GLOB_RECURSE master_srcs CONFIGURE_DEPENDS "src/master/*.cpp")
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
    ${PROJECT_NAME}::order_utils
    ${PROJECT_NAME}::errors
    ${PROJECT_NAME}::layout
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
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
  NAMESPACE ${PROJECT_NAME}::
)
ament_export_dependencies(PahoMqttCpp fmt nlohmann_json)

if(ENABLE_ROS2)
  ament_export_dependencies(vda5050_interfaces)
endif()

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
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
  include(cmake/vda5050_ament_lint_common.cmake)
  vda5050_ament_lint_common()

  find_package(ament_cmake_gmock REQUIRED)

  set(json_tests test/json_utils/test_json_utils.cpp)
  if(ENABLE_ROS2)
    list(APPEND json_tests test/json_utils/test_json_utils_ros2.cpp)
  endif()
  ament_add_gmock(test_json_utils ${json_tests})
  target_link_libraries(test_json_utils vda5050_core::json_utils)

  ament_add_gmock(test_order_utils
    test/order_utils/test_unit_order_graph_validator.cpp)
  target_link_libraries(test_order_utils vda5050_core::order_utils)

  ament_add_gmock(test_logger
    test/logger/test_unit_default_logger.cpp
    test/logger/test_unit_logger.cpp
  )
  target_link_libraries(test_logger vda5050_core::logger)

  ament_add_gmock(test_transport
    test/transport/test_unit_mqtt_client_interface.cpp
    test/transport/test_integration_paho_mqtt_client.cpp
  )
  target_link_libraries(test_transport vda5050_core::transport)

  ament_add_gmock(test_client
    test/client/test_order_types.cpp
    test/client/test_order_execution_resource.cpp
    test/client/test_agv_context.cpp
    test/client/test_order_validator.cpp
    test/client/test_order_acceptance.cpp
  )

  target_link_libraries(test_client
    vda5050_core::client
  )

  ament_add_gmock(test_execution
    test/execution/test_engine.cpp
    test/execution/test_event_queue.cpp
    test/execution/test_handler.cpp
    test/execution/test_protocol_adapter.cpp
    test/execution/test_provider.cpp
  )
  target_link_libraries(test_execution vda5050_core::execution)

  ament_add_gmock(test_master_comm
    test/master/test_integration_heartbeat.cpp
    test/master/test_integration_master_logic.cpp
    test/master/test_integration_master_mqtt.cpp
    test/master/test_integration_master_teardown.cpp
  )
  target_link_libraries(test_master_comm vda5050_core::master)

  ament_add_gmock(test_master_integration
    test/master/test_integration_master_assign_order.cpp
    test/master/test_integration_master_assign_instant_actions.cpp
    test/master/test_integration_master_mode_handling.cpp
    test/master/test_integration_master_state_timeout.cpp
    test/master/test_integration_master_broker_status.cpp
    test/master/test_master_onboard_batch.cpp
  )
  target_link_libraries(test_master_integration vda5050_core::master)

  ament_add_gmock(test_master_agv
    test/master/test_unit_agv_connection_state.cpp
    test/master/test_unit_agv_operational_state.cpp
    test/master/test_unit_agv_lifecycle.cpp
    test/master/test_unit_agv_order_lifecycle.cpp
    test/master/test_unit_agv_pose_view.cpp
  )
  target_link_libraries(test_master_agv vda5050_core::master)

  ament_add_gmock(test_master_validation
    test/master/test_unit_schema_validator.cpp
    test/master/test_unit_pre_send_validator.cpp
  )
  target_link_libraries(test_master_validation vda5050_core::master)

  ament_add_gmock(test_master_validation_order_action
    test/master/test_unit_traversability_validator.cpp
    test/master/test_unit_action_conflict_validator.cpp
    test/master/test_unit_instant_action_mode_validator.cpp
    test/master/test_unit_factsheet_alignment.cpp
  )
  target_link_libraries(test_master_validation_order_action vda5050_core::master)

  ament_add_gmock(test_layout_loader
    test/layout/test_unit_loader.cpp
  )
  target_link_libraries(test_layout_loader vda5050_core::layout)
  target_compile_definitions(test_layout_loader PRIVATE
    VDA5050_CORE__SAMPLE_LAYOUT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/layout/data/sample_layout.json")

  ament_add_gmock(test_layout_graph
    test/layout/test_unit_graph.cpp
  )
  target_link_libraries(test_layout_graph vda5050_core::layout)
  target_compile_definitions(test_layout_graph PRIVATE
    VDA5050_CORE__SAMPLE_LAYOUT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/layout/data/sample_layout.json")

  ament_add_gmock(test_master_detectors
    test/master/test_unit_state_event_detector.cpp
    test/master/test_unit_connection_event_detector.cpp
  )
  target_link_libraries(test_master_detectors vda5050_core::master)

  ament_add_gmock(test_master_event_detector
    test/master/test_unit_event_detector.cpp
  )
  target_link_libraries(test_master_event_detector vda5050_core::master)

  ament_add_gmock(test_master_order
    test/master/test_unit_order_publisher.cpp
    test/master/test_unit_order_lifecycle_manager.cpp
    test/master/test_unit_order_stitcher.cpp
  )
  target_link_libraries(test_master_order vda5050_core::master)

  ament_add_gmock(test_master_actions
    test/master/test_unit_action_factory.cpp
  )
  target_link_libraries(test_master_actions vda5050_core::master)
endif()

# Install linter macro
install(
  FILES
    cmake/vda5050_ament_lint_common.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package(
  CONFIG_EXTRAS "vda5050_core-extras.cmake"
)
