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

        // Parameter for controlling wait time at intermediate steps
        this->declare_parameter<int>("wait_time", 1.0);
        this->get_parameter("wait_time", this->wait_time);

        // Parameters for controlling end effector orientation
        this->declare_parameter<double>("orientation_x", 0.0);
        this->declare_parameter<double>("orientation_y", 1.0);
        this->declare_parameter<double>("orientation_z", 0.0);
        this->declare_parameter<double>("orientation_w", 0.0);

        RCLCPP_INFO(this->get_logger(), "Service server ready.");
    }

private:
    void handle_move_to_coordinates(const std::shared_ptr<MoveToCoordinates::Request> request,
                                    std::shared_ptr<MoveToCoordinates::Response> response) {
        // Initialize the robot to the play positon
        if (!play_position()) {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to reach play position.");
        }
        sleep_arm();

        // Handle different modes
        switch (request->mode) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Mode 0: Doing nothing.");
                // Move to the specified target position
                if (!move_to_target(request->x, request->y, request->z)) {
                    response->success = false;
                    RCLCPP_ERROR(this->get_logger(), "Failed to move to provided coordinates.");
                    break;
                }
                sleep_arm();
                response->success = true;
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "Mode 1: Drawing X symbol.");
                // Move to the specified target position
                if (!move_to_target(request->x, request->y, request->z)) {
                    response->success = false;
                    RCLCPP_ERROR(this->get_logger(), "Failed to move to provided coordinates.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Succesfully moved to provided coordinates.");
                sleep_arm();
                response->success = draw_x();
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "Mode 2: Drawing O symbol.");
                // Move to the specified target position
                if (!move_to_target(request->x, request->y, request->z)) {
                    response->success = false;
                    RCLCPP_ERROR(this->get_logger(), "Failed to move to provided coordinates.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Succesfully moved to provided coordinates.");
                sleep_arm();
                response->success = draw_o();
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "Mode 3: Drawing game board.");
                // Move to the specified target position
                if (!move_to_target(request->x, request->y, request->z)) {
                    response->success = false;
                    RCLCPP_ERROR(this->get_logger(), "Failed to move to provided coordinates.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Succesfully moved to provided coordinates.");
                sleep_arm();
                response->success = draw_grid();
                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "Mode 4: Drawing simple line.");
                response->success = draw_line();
                // Move to the specified target position
                if (!move_to_target(request->x, request->y, request->z)) {
                    response->success = false;
                    RCLCPP_ERROR(this->get_logger(), "Failed to move to provided coordinates.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Succesfully moved to provided coordinates.");
                sleep_arm();
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

        // Return to the play position
        play_position();
    }

    bool play_position() {
        move_group_->setStartStateToCurrentState();
        move_group_->setNamedTarget("play_position");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to play position.");
            return false;
        }
    }

    bool clear_position() {
        move_group_->setStartStateToCurrentState();
        move_group_->setNamedTarget("clear_board");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to play position.");
            return false;
        }
    }

    void sleep_arm() {
        RCLCPP_INFO(this->get_logger(), "Sleeping for %d seconds at current position.", this->wait_time);
        std::this_thread::sleep_for(std::chrono::seconds(this->wait_time));
        RCLCPP_INFO(this->get_logger(), "Sleep finished, continuing motion.");
    }

    bool move_to_target(double x, double y, double z) {
        // Build the target pose
        geometry_msgs::msg::Pose target_pose;
        this->get_parameter("orientation_x", target_pose.orientation.x);
        this->get_parameter("orientation_y", target_pose.orientation.y);
        this->get_parameter("orientation_z", target_pose.orientation.z);
        this->get_parameter("orientation_w", target_pose.orientation.w);
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        move_group_->setPoseTarget(target_pose);
        std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
        const int num_attempts = 5;  // Number of planning attempts

        // Generate multiple plans
        for (int i = 0; i < num_attempts; ++i) {
            moveit::planning_interface::MoveGroupInterface::Plan temp_plan;
            if (move_group_->plan(temp_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                plans.push_back(temp_plan);
            } else {
                RCLCPP_WARN(this->get_logger(), "Planning attempt %d failed", i + 1);
            }
        }

        if (plans.empty()) {
            RCLCPP_ERROR(this->get_logger(), "All planning attempts failed");
            return false;
        }

        // Function to evaluate the quality of a plan
        auto evaluatePlan = [](const moveit::planning_interface::MoveGroupInterface::Plan &plan) {
            double path_length = 0.0;
            const auto &trajectory = plan.trajectory_.joint_trajectory;
            for (size_t i = 1; i < trajectory.points.size(); ++i) {
                double segment_length = 0.0;
                for (size_t j = 0; j < trajectory.joint_names.size(); ++j) {
                    double delta = trajectory.points[i].positions[j] - trajectory.points[i-1].positions[j];
                    segment_length += delta * delta;
                }
                path_length += std::sqrt(segment_length);
            }
            return path_length;
        };

        // Evaluate all plans and find the best one
        auto best_plan = std::min_element(plans.begin(), plans.end(),
            [&](const moveit::planning_interface::MoveGroupInterface::Plan &a, 
                const moveit::planning_interface::MoveGroupInterface::Plan &b) {
                return evaluatePlan(a) < evaluatePlan(b);
            });

        // Execute the best plan
        if (move_group_->execute(*best_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute the best plan");
            return false;
        }
    }

    bool clear_board() {
        if (!clear_position()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to clear position");
            return false;
        }
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        float delta = 0.01;
        RCLCPP_INFO(this->get_logger(), "Waiting %d seconds for stabilization at erasing position.", this->wait_time);
        std::this_thread::sleep_for(std::chrono::seconds(this->wait_time));

        RCLCPP_INFO(this->get_logger(), "Performing downward motion.");
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

        if (fraction < 0.95) {
            RCLCPP_INFO(this->get_logger(), "Failed to compute a cartesian path for the symbol requested.");
            return false;            
        }

        return move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool draw_x() {
        RCLCPP_INFO(this->get_logger(), "Performing X symbol path.");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        target_pose.position.x -= this->symbol_size / 2;
        target_pose.position.y -= this->symbol_size / 2;
        waypoints.push_back(target_pose);

        target_pose.position.z -= this->drawing_depth;
        waypoints.push_back(target_pose);

        target_pose.position.x += this->symbol_size;
        target_pose.position.y += this->symbol_size;
        waypoints.push_back(target_pose);

        target_pose.position.z += this->drawing_depth;
        waypoints.push_back(target_pose);

        target_pose.position.x -= this->symbol_size;
        waypoints.push_back(target_pose);

        target_pose.position.z -= this->drawing_depth;
        waypoints.push_back(target_pose);

        target_pose.position.x += this->symbol_size;
        target_pose.position.y -= this->symbol_size;
        waypoints.push_back(target_pose);

        target_pose.position.z += this->drawing_depth;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.95) {
            RCLCPP_INFO(this->get_logger(), "Failed to compute a cartesian path for the symbol requested.");
            return false;            
        }

        return move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool draw_o() {
        RCLCPP_INFO(this->get_logger(), "Performing O symbol path.");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        double radius = this->symbol_size / 2;
        double centerX = target_pose.position.x;
        double centerY = target_pose.position.y;
        int num_points = 64;

        target_pose.position.x = centerX + radius;
        target_pose.position.z -= this->drawing_depth;
        waypoints.push_back(target_pose);

        for (int i = 0; i < num_points; ++i) {
            double angle = 2 * M_PI * i / num_points;
            target_pose.position.x = centerX + radius * std::cos(angle);
            target_pose.position.y = centerY + radius * std::sin(angle);
            waypoints.push_back(target_pose);
        }

        target_pose.position.z += this->drawing_depth;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.95) {
            RCLCPP_INFO(this->get_logger(), "Failed to compute a cartesian path for the symbol requested.");
            return false;
        }

        return move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool draw_line() {
        RCLCPP_INFO(this->get_logger(), "Performing a simple line path.");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        target_pose.position.z -= drawing_depth;
        waypoints.push_back(target_pose);

        target_pose.position.x += symbol_size;
        waypoints.push_back(target_pose);

        target_pose.position.z += drawing_depth;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.95) {
            RCLCPP_INFO(this->get_logger(), "Failed to compute a cartesian path for the symbol requested.");
            return false;
        }

        return move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool draw_grid() {
        RCLCPP_INFO(this->get_logger(), "Performing a tic-tac-toe grid path.");
        geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;

        float center_x = target_pose.position.x;
        float center_y = target_pose.position.y;
        float cell_size = grid_size / 3;

        std::vector<geometry_msgs::msg::Pose> waypoints;

        // First vertical line
        target_pose.position.x = center_x - cell_size / 2;
        target_pose.position.y = center_y - 1.5 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z -= drawing_depth;
        waypoints.push_back(target_pose);

        target_pose.position.y = center_y + 1.5 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z += drawing_depth;
        waypoints.push_back(target_pose);
        
        // Second vertical line
        target_pose.position.x = center_x + cell_size / 2;
        target_pose.position.y = center_y - 1.5 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z -= drawing_depth;
        waypoints.push_back(target_pose);

        target_pose.position.y = center_y + 1.5 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z += drawing_depth;
        waypoints.push_back(target_pose);
        
        // First horizontal line
        target_pose.position.x = center_x - 1.0 * cell_size;
        target_pose.position.y = center_y - cell_size / 2;
        waypoints.push_back(target_pose);

        target_pose.position.z -= drawing_depth;
        waypoints.push_back(target_pose);

        target_pose.position.x = center_x + 1.0 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z += drawing_depth;
        waypoints.push_back(target_pose);
      
        // Second horizontal line
        target_pose.position.x = center_x - 1.0 * cell_size;
        target_pose.position.y = center_y + cell_size / 2;
        waypoints.push_back(target_pose);

        target_pose.position.z -= drawing_depth;
        waypoints.push_back(target_pose);

        target_pose.position.x = center_x + 1.0 * cell_size;
        waypoints.push_back(target_pose);

        target_pose.position.z += drawing_depth;
        waypoints.push_back(target_pose);
    

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.95) {
            RCLCPP_INFO(this->get_logger(), "Failed to compute a cartesian path for the symbol requested.");
            return false;
        }

        return move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    rclcpp::Service<MoveToCoordinates>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::string planning_group_;
    rclcpp::Node::SharedPtr move_group_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    float symbol_size = 0.030;
    float grid_size = 0.135;
    float drawing_depth = 0.01;
    int wait_time;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveGroupService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
