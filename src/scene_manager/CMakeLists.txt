cmake_minimum_required(VERSION 3.8)
project(scene_manager)

# find necessary packages
# find_package(ament_lint_auto)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
# Leaving for now so build does not break
find_package(example_interfaces REQUIRED)

# Optional packagesm, might remove
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(scene_manager src/scene_manager.cpp)
ament_target_dependencies(scene_manager rclcpp moveit_ros_planning_interface moveit_msgs shape_msgs tf2_ros tf2_geometry_msgs example_interfaces)

install(TARGETS scene_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME}/)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
