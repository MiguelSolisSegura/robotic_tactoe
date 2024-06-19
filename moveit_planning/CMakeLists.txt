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
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToCoordinates.srv"
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

ament_export_dependencies(rosidl_default_runtime)
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
add_executable(draw_o
    src/draw_o.cpp)
target_include_directories(draw_o
    PUBLIC include)
ament_target_dependencies(draw_o
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

install(TARGETS 
	draw_o
    DESTINATION lib/${PROJECT_NAME}
)

# Move group service
add_executable(move_group_service src/move_group_service.cpp)

# Link libraries
ament_target_dependencies(move_group_service
  ament_cmake
  rclcpp_action
  interactive_markers
  moveit_ros_planning
  control_msgs
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  example_interfaces
  geometry_msgs
)

install(TARGETS
  move_group_service
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(move_group_service "${cpp_typesupport_target}")

# Add launch files
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)
