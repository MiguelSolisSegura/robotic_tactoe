#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_planning/srv/move_to_coordinates.hpp>
#include <cmath>
#include <thread>

using MoveToCoordinates = moveit_planning::srv::MoveToCoordinates; 

class MoveGroupService : public rclcpp::Node {
public:
    MoveGroupService(): Node("move_group_service") {
        // Node options
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);

        // Create a node for the MoveGroupInterface
        move_group_node_ = rclcpp::Node::make_shared("move_group_interface", node_options);

        // Initialize the executor and add the MoveGroupInterface node
        executor_.add_node(move_group_node_);
        std::thread([this]() { executor_.spin(); }).detach();

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "ur_manipulator");

        service_ = this->create_service<MoveToCoordinates>(
            "move_to_coordinates",
            std::bind(&MoveGroupService::handle_move_to_coordinates, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Service server ready.");
    }

private:
    void handle_move_to_coordinates(const std::shared_ptr<MoveToCoordinates::Request> request,
                                    std::shared_ptr<MoveToCoordinates::Response> response) {
        // Initialize the robot to the starting state
        initialize_robot();

        RCLCPP_INFO(this->get_logger(), "Sleeping for 10 seconds at playing position.");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "Continuing.");

        // Move to the specified target position
        if (request->mode != 5) {
            if (!move_to_target(request->x, request->y, request->z)) {
                response->success = false;
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Sleeping for 10 seconds at coordinates location.");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            RCLCPP_INFO(this->get_logger(), "Continuing.");
        }

        // Handle different modes
        switch (request->mode) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Mode 0: Doing nothing.");
                response->success = true;
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "Mode 1: Drawing X symbol.");
                response->success = draw_x();
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "Mode 2: Drawing O symbol.");
                response->success = draw_o();
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "Mode 3: Drawing game board.");
                response->success = draw_grid(0.03);
                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "Mode 4: Drawing simple line.");
                response->success = draw_line();
                break;
            case 5:
                RCLCPP_INFO(this->get_logger(), "Mode 5: Erasing the screen.");
                response->success = clear_board();
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid mode: %ld", request->mode);
                response->success = false;
                break;
        }

        // Return to the initial position
        RCLCPP_INFO(this->get_logger(), "Sleeping for 10 seconds before finishing.");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "Continuing.");
        move_to_play_position();
    }

    void initialize_robot() {
        move_group_->setStartStateToCurrentState();
        move_group_->setNamedTarget("play_position");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->execute(my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to play position");
        }
    }

    bool move_to_target(double x, double y, double z) {
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = 0.00;
        target_pose.orientation.y = 1.00;
        target_pose.orientation.z = 0.00;
        target_pose.orientation.w = 0.00;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->execute(my_plan);
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to target position");
            return false;
        }
    }

    void move_to_play_position() {
        move_group_->setNamedTarget("play_position");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->execute(my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to play position");
        }
    }

    bool clear_board() {
        move_group_->setNamedTarget("clear_board_real");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        float delta = 0.02;
        if (!move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to play position");
            return false;
            
        }
        move_group_->execute(my_plan);
        RCLCPP_INFO(this->get_logger(), "Waiting 10 seconds for stabilization at erasing position.");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "Continuing.");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
        target_pose.position.z -= delta;
        waypoints.push_back(target_pose);
        target_pose.position.z += delta;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        return true;
    }

    bool draw_x() {
        RCLCPP_INFO(this->get_logger(), "Drawing X symbol");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        target_pose.position.x -= 0.025;
        target_pose.position.y -= 0.025;
        waypoints.push_back(target_pose);

        target_pose.position.z -= 0.020;
        waypoints.push_back(target_pose);

        target_pose.position.x += 0.050;
        target_pose.position.y += 0.050;
        waypoints.push_back(target_pose);

        target_pose.position.z += 0.020;
        waypoints.push_back(target_pose);

        target_pose.position.x -= 0.050;
        waypoints.push_back(target_pose);

        target_pose.position.z -= 0.020;
        waypoints.push_back(target_pose);

        target_pose.position.x += 0.050;
        target_pose.position.y -= 0.050;
        waypoints.push_back(target_pose);

        target_pose.position.z += 0.020;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        move_group_->execute(trajectory);

        return fraction > 0.95;
    }

    bool draw_o() {
        RCLCPP_INFO(this->get_logger(), "Drawing O symbol");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        double radius = 0.025;
        double centerX = target_pose.position.x;
        double centerY = target_pose.position.y;
        int num_points = 64;

        target_pose.position.x = centerX + radius;
        target_pose.position.z -= 0.020;
        waypoints.push_back(target_pose);

        for (int i = 0; i < num_points; ++i) {
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

        return fraction > 0.95;
    }

    bool draw_line() {
        RCLCPP_INFO(this->get_logger(), "Drawing a simple line");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        // target_pose.position.x -= 0.025;
        // waypoints.push_back(target_pose);

        target_pose.position.z -= 0.020;
        waypoints.push_back(target_pose);

        target_pose.position.x += 0.050;
        waypoints.push_back(target_pose);

        target_pose.position.z += 0.020;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        move_group_->execute(trajectory);

        return fraction > 0.95;
    }

    bool draw_grid(double cell_size) {
        RCLCPP_INFO(this->get_logger(), "Drawing tic-tac-toe grid");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;

        double center_x = target_pose.position.x;
        double center_y = target_pose.position.y;
        double initial_z = target_pose.position.z;

        std::vector<geometry_msgs::msg::Pose> waypoints;

        // First vertical line
        target_pose.position.x = center_x - cell_size / 2;
        target_pose.position.y = center_y - 1.5 * cell_size;
        target_pose.position.z = initial_z;
        waypoints.push_back(target_pose);

        target_pose.position.z = initial_z - 0.020;
        waypoints.push_back(target_pose);

        target_pose.position.y = center_y + 1.5 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z = initial_z;
        waypoints.push_back(target_pose);

        // Second vertical line
        target_pose.position.x = center_x + cell_size / 2;
        target_pose.position.y = center_y - 1.5 * cell_size;
        target_pose.position.z = initial_z;
        waypoints.push_back(target_pose);

        target_pose.position.z = initial_z - 0.020;
        waypoints.push_back(target_pose);

        target_pose.position.y = center_y + 1.5 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z = initial_z;
        waypoints.push_back(target_pose);

        // First horizontal line
        target_pose.position.x = center_x - 1.5 * cell_size;
        target_pose.position.y = center_y - cell_size / 2;
        target_pose.position.z = initial_z;
        waypoints.push_back(target_pose);

        target_pose.position.z = initial_z - 0.020;
        waypoints.push_back(target_pose);

        target_pose.position.x = center_x + 1.5 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z = initial_z;
        waypoints.push_back(target_pose);

        // Second horizontal line
        target_pose.position.x = center_x - 1.5 * cell_size;
        target_pose.position.y = center_y + cell_size / 2;
        target_pose.position.z = initial_z;
        waypoints.push_back(target_pose);

        target_pose.position.z = initial_z - 0.020;
        waypoints.push_back(target_pose);

        target_pose.position.x = center_x + 1.5 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z = initial_z;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        move_group_->execute(trajectory);

        return fraction > 0.95;
    }

    rclcpp::Service<MoveToCoordinates>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::string planning_group_;
    rclcpp::Node::SharedPtr move_group_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveGroupService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
