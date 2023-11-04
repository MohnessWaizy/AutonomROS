cmake_minimum_required(VERSION 3.5)
project(astra_pro)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(iceoryx_msg REQUIRED)
find_package(Astra REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(frame_listener src/astra_pro_frame_listener.cpp)

include_directories(include)
include_directories(${ASTRA_INCLUDE_DIR})
add_executable(astra_pro_publisher src/astra_pro_publisher.cpp)
target_link_libraries(astra_pro_publisher frame_listener ${ASTRA_LIBRARIES})
ament_target_dependencies(astra_pro_publisher rclcpp lifecycle_msgs rclcpp_lifecycle sensor_msgs iceoryx_msg)

install(TARGETS
  astra_pro_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_lint_cmake
    ament_cmake_cpplint
  )
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_lint_cmake(CMakeLists.txt) # ignore FindAstra.cmake
  ament_cpplint(FILTERS -legal/copyright)
endif()

# Install launch files.
install(DIRECTORY
  launch params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
