cmake_minimum_required(VERSION 3.8)
project(capella_ros_dock)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(capella_ros_dock_msgs REQUIRED)
find_package(capella_ros_msg REQUIRED)
find_package(capella_ros_service_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(aruco_msgs)
find_package(charge_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

set(dependencies
  angles
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_geometry_msgs
  tf2_ros
  Boost
  capella_ros_dock_msgs
  capella_ros_msg
  capella_ros_service_interfaces
  std_srvs
  OpenCV
  image_transport
  aruco_msgs
  charge_manager_msgs
  std_msgs
  nav2_costmap_2d
  ament_cmake
  nav2_common
  nav2_msgs
)

nav2_package()

# Hazards vector publisher
add_library(hazards_publisher_lib  SHARED)
target_sources(hazards_publisher_lib 
  PRIVATE 
  src/hazards_vector_publisher.cpp)
target_include_directories(hazards_publisher_lib PUBLIC include)
ament_target_dependencies(hazards_publisher_lib ${dependencies})
rclcpp_components_register_node(hazards_publisher_lib
    PLUGIN "capella_ros_dock::HazardsVectorPublisher"
    EXECUTABLE hazards_vector_publisher)

# Robot status publisher
# add_library(robot_state_lib SHARED)
# target_sources(robot_state_lib PRIVATE src/robot_state.cpp)
# target_include_directories(robot_state_lib PUBLIC include)
# ament_target_dependencies(robot_state_lib ${dependencies})
# rclcpp_components_register_node(robot_state_lib
#     PLUGIN "capella_ros_dock::RobotState"
#     EXECUTABLE robot_state)

#Motion control node
add_library(motion_control_lib SHARED)
target_sources(
  motion_control_lib
  PRIVATE
  src/motion_control_node.cpp
  src/docking_behavior.cpp
  # src/reflex_behavior.cpp
  # src/drive_goal_behaviors.cpp
  # src/wall_follow_behavior.cpp
  # src/wall_follow_states.cpp
)
target_include_directories(motion_control_lib PUBLIC include)
ament_target_dependencies(motion_control_lib
  ${dependencies})
rclcpp_components_register_node(motion_control_lib
  PLUGIN "capella_ros_dock::MotionControlNode"
  EXECUTABLE motion_control)

# test_dock node for test succcess rate
add_executable(test_dock src/test_dock_node.cpp src/test_dock.cpp)
target_include_directories(test_dock PUBLIC include)
ament_target_dependencies(test_dock
  ${dependencies}
)
install(TARGETS test_dock 
  DESTINATION lib/${PROJECT_NAME})

# for point_cloud test
add_executable(point_cloud_test src/point_cloud_test.cpp)
target_include_directories(point_cloud_test PUBLIC include)
ament_target_dependencies(point_cloud_test ${dependencies})
install(TARGETS point_cloud_test DESTINATION lib/${PROJECT_NAME})

# camera_point_cloud_process library
add_library(camera_point_cloud_process_lib SHARED)
target_sources(camera_point_cloud_process_lib
  PRIVATE
  src/camera_point_cloud_process.cpp
)

target_include_directories(camera_point_cloud_process_lib PUBLIC include)
ament_target_dependencies(camera_point_cloud_process_lib
  ${dependencies})
rclcpp_components_register_node(camera_point_cloud_process_lib
  PLUGIN "capella_ros_dock::CameraPointCloudProcess" 
  EXECUTABLE camera_point_cloud_process)

# manual_dock library
add_library(manual_dock_lib SHARED)
target_sources(manual_dock_lib
  PRIVATE
  src/manual_dock.cpp
)

target_include_directories(manual_dock_lib PUBLIC include)
ament_target_dependencies(manual_dock_lib
  ${dependencies})
rclcpp_components_register_node(manual_dock_lib
  PLUGIN "capella_ros_dock::ManualDock" 
  EXECUTABLE manual_dock)

# coord_optimize library
add_library(coord_optimize SHARED src/coord_optimize.cpp)
target_include_directories(coord_optimize PUBLIC include)
ament_target_dependencies(coord_optimize ${dependencies})
rclcpp_components_register_node(coord_optimize PLUGIN "capella_ros_dock::CoordOptimize" EXECUTABLE coord_optimize_node)

############install libraries
set(libraries_names
  hazards_publisher_lib
  motion_control_lib
  camera_point_cloud_process_lib
  manual_dock_lib
  coord_optimize
  # robot_state_lib
)
install(TARGETS ${libraries_names}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})


ament_package()
