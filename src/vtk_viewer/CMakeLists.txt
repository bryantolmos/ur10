cmake_minimum_required(VERSION 3.8)
project(vtk_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_eigen REQUIRED)

# Find Qt6
# This enables AUTOMOC, AUTOUIC, AUTORCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Find VTK
# This finds the VTK libraries
find_package(VTK REQUIRED)

# Add our executable
add_executable(vtk_node src/main.cpp src/mainwindow.cpp)

# Link all the libraries
ament_target_dependencies(vtk_node 
rclcpp 
moveit_msgs
moveit_core
moveit_ros_planning_interface
geometric_shapes
ament_index_cpp
tf2_eigen
)

# Link Qt6 and VTK (using the "plain" signature)
target_link_libraries(vtk_node
  Qt5::Widgets
  ${VTK_LIBRARIES}
)

# Install our executable
install(TARGETS vtk_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}  
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
