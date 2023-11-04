cmake_minimum_required(VERSION 3.5)
project(autonomros_lane_detection)

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


set(lib_name ${PROJECT_NAME}_core)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge)
find_package(image_transport)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree)
find_package(nav2_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(iceoryx_msg REQUIRED)
set(PahoMqttCpp_LIBRARIES PahoMqttCpp::paho-mqttpp3)



#find_package(nav2_behaviors REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


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


set(dep_pkgs
    nav2_costmap_2d
    pluginlib
    rclcpp_action
    rclcpp_lifecycle
    nav2_util
    nav2_msgs
    nav_msgs
    nav2_core
    behaviortree_cpp_v3
    geometry_msgs
    nav2_behavior_tree)

add_library(${lib_name} SHARED
    src/lane_layer.cpp
    src/lane_controller.cpp
    src/intersection_checker.cpp
    src/is_intersection_free_bt.cpp)
    #src/V2I_Node_Test.cpp)

list(APPEND plugin_libs ${lib_name})


ament_target_dependencies(${lib_name} ${dependencies})
    
include_directories(include)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(lane_detector src/lane_detector.cpp)
add_executable(Intersection_Checker src/intersection_checker.cpp)


#add_executable(lane_layer src/lane_layer.cpp)
ament_target_dependencies(lane_detector rclcpp std_msgs sensor_msgs OpenCV pcl_conversions cv_bridge image_transport iceoryx_msg)
ament_target_dependencies(Intersection_Checker rclcpp tf2 tf2_ros geometry_msgs std_srvs)
#ament_target_dependencies(check std_msgs action_msgs nav2_msgs)
ament_target_dependencies(${lib_name} rclcpp std_msgs sensor_msgs nav2_core iceoryx_msg)



#rosidl_target_interfaces(lane_detector
#      ${PROJECT_NAME} "rosidl_typesupport_cpp")



install(TARGETS
  lane_detector Intersection_Checker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch data scripts params
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${lib_name}
        DESTINATION lib)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${lib_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
install(FILES params/behaviour_plugin.xml DESTINATION share/${PROJECT_NAME})

target_link_libraries(lane_detector ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})
target_link_libraries(${lib_name} ${PCL_LIBRARIES} ${PahoMqttCpp_LIBRARIES})
target_link_libraries(Intersection_Checker ${PahoMqttCpp_LIBRARIES})


pluginlib_export_plugin_description_file(nav2_costmap_2d params/lane_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

pluginlib_export_plugin_description_file(nav2_core params/lane_controller_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core params/behaviour_plugin.xml)

# Install Python executables
install(PROGRAMS
  scripts/detect_lanes.py 
  DESTINATION lib/${PROJECT_NAME}
)



#rosidl_target_interfaces(check ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_libraries(
  ${lib_name}
  ${plugin_libs}
)

ament_package()
