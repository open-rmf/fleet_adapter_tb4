cmake_minimum_required(VERSION 3.8)
project(fleet_adapter_tb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
set(dep_pkgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  rmf_fleet_adapter
  rmf_traffic_ros2
  tf2_geometry_msgs
  tf2
  tf2_ros
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

#===============================================================================
# add_library(fleet_adapter SHARED src/FleetAdapter.cpp)
add_executable(fleet_adapter src/FleetAdapter.cpp)

target_link_libraries(fleet_adapter
  PUBLIC
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${nav2_msgs_LIBRARIES}
    rclcpp::rclcpp
    ${rclcpp_action_LIBRARIES}
    ${rclcpp_components_LIBRARIES}
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_traffic_ros2::rmf_traffic_ros2
    tf2_geometry_msgs::tf2_geometry_msgs
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
)

target_include_directories(fleet_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${nav2_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_action_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_fleet_adapter_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

target_compile_features(fleet_adapter INTERFACE cxx_std_17)

# rclcpp_components_register_node(fleet_adapter
#   PLUGIN "FleetAdapter"
#   EXECUTABLE full_control
#   EXECUTOR MultiThreadedExecutor)

#===============================================================================
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS fleet_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
