cmake_minimum_required(VERSION 3.8)
project(ur10_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# 1. Path Generator Node
add_executable(path_generator_node src/path_generator_node.cpp)
ament_target_dependencies(path_generator_node 
  rclcpp geometry_msgs tf2 tf2_geometry_msgs
)
target_link_libraries(path_generator_node yaml-cpp)

# 2. Trajectory Supervisor Node
add_executable(trajectory_supervisor_node src/trajectory_supervisor_node.cpp)
ament_target_dependencies(trajectory_supervisor_node 
  rclcpp geometry_msgs moveit_msgs moveit_ros_planning_interface moveit_visual_tools
)

# 3. Safety Validator Node
add_executable(safety_validator_node src/safety_validator_node.cpp)
ament_target_dependencies(safety_validator_node 
  rclcpp moveit_msgs
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS 
    path_generator_node
    trajectory_supervisor_node
    safety_validator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
