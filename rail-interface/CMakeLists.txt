cmake_minimum_required(VERSION 3.20)
project(rail_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

generate_parameter_library(
  rail_drive_parameters
  src/rail-drive.yaml
)

add_executable(rail-drive src/rail-drive.cpp)
target_compile_features(rail-drive PUBLIC cxx_std_17)
target_include_directories(rail-drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rail-drive rail_drive_parameters)
ament_target_dependencies(rail-drive
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rail-drive rail_drive_parameters
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
