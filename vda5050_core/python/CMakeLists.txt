find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)
find_package(pybind11_json REQUIRED)

pybind11_add_module(_core
  src/core.cpp
  src/types.cpp
  src/client.cpp
  src/master.cpp
  src/rmf_migration.cpp
  src/transport.cpp
)

target_link_libraries(_core
  PRIVATE
    vda5050_core::logger
    vda5050_core::client
    vda5050_core::master
    pybind11_json
)

target_include_directories(_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(SKBUILD)
  # Install for scikit-build-core
  install(TARGETS
    _core
    DESTINATION .
  )

  # Manual RPATH patching
  if(APPLE)
    set(origin_token "@loader_path")
  else()
    set(origin_token "$ORIGIN")
  endif()
  set_target_properties(_core
    PROPERTIES
      INSTALL_RPATH "${origin_token}/lib"
  )
  set_target_properties(
      vda5050_logger
      vda5050_validation
      vda5050_transport
      vda5050_execution
      vda5050_client
      vda5050_layout
      vda5050_master
    PROPERTIES
      INSTALL_RPATH "${origin_token}"
  )
else()
  find_package(ament_cmake_python QUIET)

  if(NOT ament_cmake_python_FOUND)
    message(WARNING
      "Python build with CMake only detected, some functions are not supported. (For example, `pip show vda5050_core`)\n"
      "Switch to using scikit-build-core or ament_cmake_python instead!"
    )

    # setup installation path for ament_cmake hooks
    set(PYTHON_INSTALL_DIR "local/lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages")

    install(
      DIRECTORY ${PROJECT_NAME}/
      DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
    )

    # TODO(anyone): egg installation to make package detectable by pip
  else()
    ament_python_install_package(${PROJECT_NAME})
  endif()

  install(TARGETS
    _core
    DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  )

  # export python path as hooks used by ament_cmake
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/pythonpath.dsv "prepend-non-duplicate;PYTHONPATH;${PYTHON_INSTALL_DIR}")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/pythonpath.dsv DESTINATION share/${PROJECT_NAME}/hook)
endif()
