#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node =
        rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "ur_manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    const moveit::core::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),
              move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // Declare parameters
    double orientation_x, orientation_y, orientation_z, orientation_w;
    double position_x, position_y, position_z;

    // Get parameter values
    move_group_node->get_parameter("orientation_x", orientation_x);
    move_group_node->get_parameter("orientation_y", orientation_y);
    move_group_node->get_parameter("orientation_z", orientation_z);
    move_group_node->get_parameter("orientation_w", orientation_w);
    move_group_node->get_parameter("position_x", position_x);
    move_group_node->get_parameter("position_y", position_y);
    move_group_node->get_parameter("position_z", position_z);

    RCLCPP_INFO(LOGGER, "position_x: %.3f", position_x);

    // Define goal
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = orientation_x;
    target_pose1.orientation.y = orientation_y;
    target_pose1.orientation.z = orientation_z;
    target_pose1.orientation.w = orientation_w;
    target_pose1.position.x = position_x;
    target_pose1.position.y = position_y;
    target_pose1.position.z = position_z;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success =
        (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Starting motion");
    move_group.execute(my_plan);

    rclcpp::shutdown();
    return 0;
}
