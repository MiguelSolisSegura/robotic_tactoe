#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

// Define a static logger for logging information
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
    // Initialize ROS 2 communication
    rclcpp::init(argc, argv);

    // Create a node with options to automatically declare parameters from overrides
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // Create a single-threaded executor and add the node to it
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);

    // Spin the executor in a separate thread
    std::thread([&executor]() { executor.spin(); }).detach();

    // Define the planning group for the arm
    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

    // Initialize the MoveGroupInterface for the arm
    moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);

    // Get the joint model group for the arm
    const moveit::core::JointModelGroup *joint_model_group_arm =
        move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

    // Get the current state of the robot arm
    moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);

    // Copy the current joint positions of the arm
    std::vector<double> joint_group_positions_arm;
    current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

    // Set the starting state to the current state of the robot arm
    move_group_arm.setStartStateToCurrentState();

    // Initial position setup
    RCLCPP_INFO(LOGGER, "Setting initial position");

    // Got to the intial pose for playing
    joint_group_positions_arm[0] = -1.5707;  
    joint_group_positions_arm[1] = -0.7853; 
    joint_group_positions_arm[2] =  0.0000;  
    joint_group_positions_arm[3] =  0.7853;
    joint_group_positions_arm[4] =  0.0000;  
    joint_group_positions_arm[5] =  0.0000;

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute the planned motion to the initial position
    move_group_arm.execute(my_plan_arm);

    // Grid position setup
    RCLCPP_INFO(LOGGER, "Going to grid location");

    // Define the target pose for the grid position
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = 0.00;
    target_pose1.orientation.y = 0.00;
    target_pose1.orientation.z = 0.00;
    target_pose1.orientation.w = 1.00;
    target_pose1.position.x = 0.160;
    target_pose1.position.y = 0.000;
    target_pose1.position.z = 0.060;

    move_group_arm.setPoseTarget(target_pose1);

    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute the planned motion to the grid position
    move_group_arm.execute(my_plan_arm);

    // X motion definiton
    RCLCPP_INFO(LOGGER, "Drawing an X symbol");

    // Define waypoints for the X motion
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // First line
    target_pose1.position.x -= 0.025;
    target_pose1.position.y -= 0.025;
    waypoints.push_back(target_pose1);

    target_pose1.position.z -= 0.020;
    waypoints.push_back(target_pose1);

    target_pose1.position.x += 0.050;
    target_pose1.position.y += 0.050;
    waypoints.push_back(target_pose1);

    target_pose1.position.z += 0.020;
    waypoints.push_back(target_pose1);

    // Second line
    target_pose1.position.x -= 0.050;
    waypoints.push_back(target_pose1);

    target_pose1.position.z -= 0.020;
    waypoints.push_back(target_pose1);

    target_pose1.position.x += 0.050;
    target_pose1.position.y -= 0.050;
    waypoints.push_back(target_pose1);

    target_pose1.position.z += 0.020;
    waypoints.push_back(target_pose1);

    // Compute the Cartesian path for the waypoints
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // Execute the computed Cartesian path
    move_group_arm.execute(trajectory);

    // Get back to the playing position
    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute the planned motion to the initial position
    move_group_arm.execute(my_plan_arm);


    // Shutdown ROS 2 communication
    rclcpp::shutdown();
    return 0;
}
