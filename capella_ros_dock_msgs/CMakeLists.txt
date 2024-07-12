cmake_minimum_required(VERSION 3.8)
project(capella_ros_dock_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DockStatus.msg"
  "msg/HazardDetection.msg"
  "msg/HazardDetectionVector.msg"
  "msg/IrIntensity.msg"
  "msg/IrIntensityVector.msg"
  "msg/IrOpcode.msg"
  "msg/KidnapStatus.msg"
  "msg/StopStatus.msg"
  "msg/WheelStatus.msg"
  "msg/WheelVels.msg"
  "msg/ChargerContactConditionType.msg"
  "srv/EStop.srv"
  "srv/RobotPower.srv"
  "action/Dock.action"
  "action/DriveArc.action"
  "action/DriveDistance.action"
  "action/NavigateToPosition.action"
  "action/RotateAngle.action"
  "action/Undock.action"
  "action/WallFollow.action"
  DEPENDENCIES nav_msgs
  DEPENDENCIES sensor_msgs
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
