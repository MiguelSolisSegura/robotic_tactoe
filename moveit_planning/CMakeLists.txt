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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)


# Add the service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToCoordinates.srv"
  "srv/GetBoardState.srv"
  "srv/ComputeBestMove.srv"
  "srv/MakeMove.srv"
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

# tictactoe main node
add_executable(tic_tac_toe src/tic_tac_toe.cpp)

ament_target_dependencies(tic_tac_toe
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
  std_srvs
)

install(TARGETS
  tic_tac_toe
  DESTINATION lib/${PROJECT_NAME}
)

# Interfaces definiton
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(move_group_service "${cpp_typesupport_target}")
target_link_libraries(tic_tac_toe "${cpp_typesupport_target}")

# Add launch files
install(DIRECTORY 
	launch
	config
	models
    DESTINATION share/${PROJECT_NAME}
)
