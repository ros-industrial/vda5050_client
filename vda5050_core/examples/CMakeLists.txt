add_executable(custom_base execution/custom_base.cpp)
target_link_libraries(custom_base PUBLIC
  ${PROJECT_NAME}::execution
  ${PROJECT_NAME}::logger
)

add_executable(provider_example execution/provider_example.cpp)
target_link_libraries(provider_example PUBLIC
  ${PROJECT_NAME}::execution
  ${PROJECT_NAME}::logger
)

add_executable(engine_example execution/engine_example.cpp)
target_link_libraries(engine_example PUBLIC
  ${PROJECT_NAME}::execution
  ${PROJECT_NAME}::logger
)

add_executable(handler_integration execution/handler_integration.cpp)
target_link_libraries(handler_integration PUBLIC
  ${PROJECT_NAME}::execution
  ${PROJECT_NAME}::logger
)

add_executable(adapter_example client/adapter_example.cpp)
target_link_libraries(adapter_example PUBLIC
  ${PROJECT_NAME}::client
  ${PROJECT_NAME}::transport
  ${PROJECT_NAME}::logger
)

add_executable(order_publisher master/order_publisher.cpp)
target_link_libraries(order_publisher PUBLIC
  ${PROJECT_NAME}::transport
  ${PROJECT_NAME}::logger
  ${PROJECT_NAME}::execution
)

add_executable(master_example master/master_example.cpp)
target_link_libraries(master_example PUBLIC
  ${PROJECT_NAME}::master
  ${PROJECT_NAME}::transport
  ${PROJECT_NAME}::logger
)

install(TARGETS
    custom_base
    provider_example
    engine_example
    handler_integration
    adapter_example
    order_publisher
    master_example
  DESTINATION lib/${PROJECT_NAME}
)
