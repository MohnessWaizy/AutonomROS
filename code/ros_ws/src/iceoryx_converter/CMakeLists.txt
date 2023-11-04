cmake_minimum_required(VERSION 3.8)
project(iceoryx_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(iceoryx_msg REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(iceoryx_converter_mono src/iceoryx_converter_mono.cpp)
ament_target_dependencies(iceoryx_converter_mono rclcpp sensor_msgs iceoryx_msg)

add_executable(iceoryx_converter_point_cloud src/iceoryx_converter_point_cloud.cpp)
ament_target_dependencies(iceoryx_converter_point_cloud rclcpp sensor_msgs iceoryx_msg)

add_executable(iceoryx_converter_rgb src/iceoryx_converter_rgb.cpp)
ament_target_dependencies(iceoryx_converter_rgb rclcpp sensor_msgs iceoryx_msg)

add_executable(iceoryx_converter_lines src/iceoryx_converter_lines.cpp)
ament_target_dependencies(iceoryx_converter_lines rclcpp sensor_msgs iceoryx_msg)

add_executable(iceoryx_hz_point_cloud src/iceoryx_hz_point_cloud.cpp)
ament_target_dependencies(iceoryx_hz_point_cloud rclcpp sensor_msgs iceoryx_msg)

install(TARGETS
  iceoryx_converter_rgb iceoryx_converter_point_cloud iceoryx_converter_mono iceoryx_converter_lines iceoryx_hz_point_cloud
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
