cmake_minimum_required(VERSION 3.8)
project(moveit_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

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

# Test end effector space
add_executable(test_trajectory
    src/test_trajectory.cpp)
target_include_directories(test_trajectory
    PUBLIC include)
ament_target_dependencies(test_trajectory
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

install(TARGETS 
	test_trajectory
    DESTINATION lib/${PROJECT_NAME}
)

# Test joint space
add_executable(test_joint
    src/test_joint.cpp)
target_include_directories(test_joint
    PUBLIC include)
ament_target_dependencies(test_joint
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

install(TARGETS 
	test_joint
    DESTINATION lib/${PROJECT_NAME}
)

# Draw 'x' shape
add_executable(draw_x
    src/draw_x.cpp)
target_include_directories(draw_x
    PUBLIC include)
ament_target_dependencies(draw_x
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

install(TARGETS 
	draw_x
    DESTINATION lib/${PROJECT_NAME}
)

# Draw 'o' shape

# Add launch files
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)