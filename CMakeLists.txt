cmake_minimum_required(VERSION 3.20)
project(easynav_experiments)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(easynav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/scan_mode_bridge.cpp
  src/benchmark_evaluator.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  ${action_msgs_TARGETS}
  ${easynav_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

add_executable(scan_mode_bridge
  src/scan_mode_bridge_main.cpp
)
target_link_libraries(scan_mode_bridge
  ${PROJECT_NAME}
)

add_executable(benchmark_evaluator
  src/benchmark_evaluator_main.cpp
)
target_link_libraries(benchmark_evaluator
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch config maps params results
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  scan_mode_bridge
  benchmark_evaluator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scan_mode_bridge
    test/test_scan_mode_bridge.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  target_link_libraries(test_scan_mode_bridge
    ${PROJECT_NAME}
  )
  target_link_libraries(test_scan_mode_bridge
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_srvs_TARGETS}
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(action_msgs easynav_interfaces geometry_msgs rclcpp sensor_msgs std_srvs)

ament_package()
