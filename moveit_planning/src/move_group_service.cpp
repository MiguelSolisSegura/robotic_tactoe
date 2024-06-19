#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_planning/srv/move_to_coordinates.hpp>
#include <cmath>

using MoveToCoordinates = moveit_planning::srv::MoveToCoordinates; 

class MoveGroupService : public rclcpp::Node {
public:
MoveGroupService(): Node("move_group_service"){
    // Node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // Create a node for the MoveGroupInterface
    move_group_node_ = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // Initialize the executor and add the MoveGroupInterface node
    executor_.add_node(move_group_node_);
    std::thread([this]() { executor_.spin(); }).detach();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "ur_manipulator");

    // joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup("ur_manipulator");

    service_ = this->create_service<MoveToCoordinates>(
        "move_to_coordinates",
        std::bind(&MoveGroupService::handle_move_to_coordinates, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service server ready.");
}

private:
void handle_move_to_coordinates(const std::shared_ptr<MoveToCoordinates::Request> request,
                                std::shared_ptr<MoveToCoordinates::Response> response){
    move_group_->setStartStateToCurrentState();

    RCLCPP_INFO(this->get_logger(), "Setting initial position");
    move_group_->setNamedTarget("play_position");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
    move_group_->execute(my_plan);
    }
    else
    {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to play position");
    response->success = false;
    return;
    }

    RCLCPP_INFO(this->get_logger(), "Going to target position");
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 0.00;
    target_pose.orientation.y = 0.00;
    target_pose.orientation.z = 0.00;
    target_pose.orientation.w = 1.00;
    target_pose.position.x = request->x;
    target_pose.position.y = request->y;
    target_pose.position.z = request->z + 0.02;
    move_group_->setPoseTarget(target_pose);

    success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
    move_group_->execute(my_plan);
    }
    else
    {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to target position");
    response->success = false;
    return;
    }

    if (!request->perform_o_pattern)
    {
    RCLCPP_INFO(this->get_logger(), "Drawing an O symbol");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    double radius = 0.025;
    double centerX = target_pose.position.x;
    double centerY = target_pose.position.y;
    int num_points = 64;

    target_pose.position.x = centerX + radius;
    target_pose.position.z -= 0.020;
    waypoints.push_back(target_pose);

    for (int i = 0; i < num_points; ++i)
    {
        double angle = 2 * M_PI * i / num_points;
        target_pose.position.x = centerX + radius * std::cos(angle);
        target_pose.position.y = centerY + radius * std::sin(angle);
        waypoints.push_back(target_pose);
    }

    target_pose.position.z += 0.020;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    move_group_->execute(trajectory);
    }

    move_group_->setNamedTarget("play_position");
    success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
    move_group_->execute(my_plan);
    response->success = true;
    }
    else
    {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to play position");
    response->success = false;
    }
}

rclcpp::Service<MoveToCoordinates>::SharedPtr service_;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
// const moveit::core::JointModelGroup *joint_model_group_;
std::string planning_group_;
rclcpp::Node::SharedPtr move_group_node_;
rclcpp::executors::SingleThreadedExecutor executor_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveGroupService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
