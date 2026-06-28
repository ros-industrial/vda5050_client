cmake_minimum_required(VERSION 3.8)
project(vda5050_master_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(vda5050_core REQUIRED)
find_package(vda5050_interfaces REQUIRED)

include(GNUInstallDirs)

# --- rosidl interfaces -------------------------------------------------------
# Topic/message layer only — no synchronous service (.srv) definitions.
set(msg_files
  msg/AssignOrderRequest.msg
  msg/AssignmentResult.msg
  msg/DeviceStatus.msg
  msg/OrderStatus.msg
  msg/PoseView.msg
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES vda5050_interfaces builtin_interfaces
  ADD_LINTER_TESTS
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- library -----------------------------------------------------------------
file(GLOB_RECURSE ros2_srcs "src/*.cpp")
add_library(${PROJECT_NAME}_lib SHARED ${ros2_srcs})
target_compile_definitions(${PROJECT_NAME}_lib PUBLIC ENABLE_ROS2)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  ${cpp_typesupport_target}
  vda5050_core::master
  vda5050_core::types
  vda5050_core::json_utils
  vda5050_core::execution
  vda5050_core::transport
  vda5050_core::logger
  vda5050_core::errors
  vda5050_core::order_utils
  fmt::fmt
  rclcpp::rclcpp
  ${vda5050_interfaces_TARGETS}
  ${builtin_interfaces_TARGETS}
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

# --- install -----------------------------------------------------------------
install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rosidl_default_runtime builtin_interfaces vda5050_core
  vda5050_interfaces)

# --- tests -------------------------------------------------------------------
if(BUILD_TESTING)
  vda5050_ament_lint_common()
  find_package(ament_cmake_gmock REQUIRED)

  set(_next_domain 41)
  macro(_add_ros2_test name src)
    ament_add_gmock(${name} ${src})
    target_link_libraries(${name}
      ${PROJECT_NAME}_lib
      rclcpp::rclcpp
      ${vda5050_interfaces_TARGETS}
    )
    set_target_properties(${name} PROPERTIES
      BUILD_RPATH_USE_ORIGIN TRUE
      INSTALL_RPATH_USE_LINK_PATH TRUE)
    set_tests_properties(${name} PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=${_next_domain};RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
    math(EXPR _next_domain "${_next_domain} + 1")
  endmacro()

  _add_ros2_test(test_device_status_publisher    test/unit/test_unit_device_status_publisher.cpp)
  _add_ros2_test(test_order_status_publisher     test/unit/test_unit_order_status_publisher.cpp)
  _add_ros2_test(test_pose_view_publisher        test/unit/test_unit_pose_view_publisher.cpp)
  _add_ros2_test(test_assignment_result_publisher
                                                 test/unit/test_unit_assignment_result_publisher.cpp)
  _add_ros2_test(test_assign_order_request_subscriber
                                                 test/unit/test_unit_assign_order_request_subscriber.cpp)

  # Pure function — no DDS env needed.
  ament_add_gmock(test_order_status_builder test/unit/test_unit_order_status_builder.cpp)
  target_link_libraries(test_order_status_builder
    ${PROJECT_NAME}_lib
    rclcpp::rclcpp
    ${vda5050_interfaces_TARGETS}
  )
  set_target_properties(test_order_status_builder PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE)

  # Pure function — no DDS env needed.
  ament_add_gmock(test_pose_view_builder test/unit/test_unit_pose_view_builder.cpp)
  target_link_libraries(test_pose_view_builder
    ${PROJECT_NAME}_lib
    rclcpp::rclcpp
    ${vda5050_interfaces_TARGETS}
  )
  set_target_properties(test_pose_view_builder PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE)

  # Pure header-only helper — no DDS env, no library link needed.
  ament_add_gmock(test_ros2_topic_naming test/unit/test_unit_ros2_topic_naming.cpp)
  target_include_directories(test_ros2_topic_naming PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
