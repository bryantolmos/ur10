cmake_minimum_required(VERSION 3.8)
project(ur10_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(ur10_planner_lib
  src/waypoint_pub.cpp
)

ament_target_dependencies(
  ur10_planner_lib
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(lifecycle_planner 
  src/lifecycle_planner.cpp
)

target_link_libraries(lifecycle_planner ur10_planner_lib)

ament_target_dependencies(
  lifecycle_planner
  rclcpp
  rclcpp_lifecycle
  std_msgs
  moveit_ros_move_group
  moveit_ros_planning_interface
  moveit_msgs
  moveit_visual_tools
  tf2_geometry_msgs
  tf2
  geometry_msgs
  std_srvs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS lifecycle_planner ur10_planner_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
