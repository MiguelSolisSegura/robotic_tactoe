#include "rclcpp/rclcpp.hpp"
#include "moveit_planning/srv/get_board_state.hpp"
#include "moveit_planning/srv/compute_best_move.hpp"
#include "moveit_planning/srv/move_to_coordinates.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;
using namespace std::placeholders;

class TicTacToeOrchestrator : public rclcpp::Node {
public:
    TicTacToeOrchestrator() : Node("tictactoe_orchestrator") {
        // Create a callback group
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Initialize clients
        get_board_state_client_ = this->create_client<moveit_planning::srv::GetBoardState>("get_board_state", rmw_qos_profile_services_default, callback_group_);
        compute_best_move_client_ = this->create_client<moveit_planning::srv::ComputeBestMove>("compute_best_move", rmw_qos_profile_services_default, callback_group_);
        move_to_coordinates_client_ = this->create_client<moveit_planning::srv::MoveToCoordinates>("move_to_coordinates", rmw_qos_profile_services_default, callback_group_);
        
        // Initialize services
        ask_for_next_move_service_ = this->create_service<std_srvs::srv::Trigger>("ask_for_next_move", std::bind(&TicTacToeOrchestrator::ask_for_next_move_callback, this, _1, _2), rmw_qos_profile_services_default, callback_group_);
        start_new_game_service_ = this->create_service<std_srvs::srv::Trigger>("start_new_game", std::bind(&TicTacToeOrchestrator::start_new_game_callback, this, _1, _2));

        // Load mapping from sim_config.txt
        load_coordinates_mapping();
        RCLCPP_INFO(this->get_logger(), "Main node tic-tac-toe orchestrator started.");
    }

private:
    rclcpp::Client<moveit_planning::srv::GetBoardState>::SharedPtr get_board_state_client_;
    rclcpp::Client<moveit_planning::srv::ComputeBestMove>::SharedPtr compute_best_move_client_;
    rclcpp::Client<moveit_planning::srv::MoveToCoordinates>::SharedPtr move_to_coordinates_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ask_for_next_move_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_new_game_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::map<int, std::array<double, 3>> coordinates_mapping_;

    void load_coordinates_mapping() {
        // Load coordinates from sim_config.txt and populate coordinates_mapping_
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("moveit_planning");
        std::string config_file = package_share_directory + "/config/sim_config.txt"; 

        std::ifstream file(config_file);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open sim_config.txt");
            return;
        }

        int move;
        double x, y, z;
        while (file >> move >> x >> y >> z) {
            coordinates_mapping_[move] = {x, y, z};
        }
        RCLCPP_INFO(this->get_logger(), "Loaded coordinates mapping from sim_config.txt.");
    }

    void ask_for_next_move_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Avoid unused parameter warning
        (void)request;  
        RCLCPP_INFO(this->get_logger(), "Received request to determine and perform the next move.");
        // Create a request for /get_board_state service
        auto get_board_state_request = std::make_shared<moveit_planning::srv::GetBoardState::Request>();
        // Create a future result for the response and send request
        auto get_board_state_future = get_board_state_client_->async_send_request(get_board_state_request);
        // Handle response
        auto status = get_board_state_future.wait_for(3s);
        std::array<int, 9> board_state;
        if (status == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Received board state.");
            auto get_board_state_response = get_board_state_future.get();
            RCLCPP_INFO(this->get_logger(), "Response is:\n%s", get_board_state_response->board_ascii.c_str());
            board_state = get_board_state_response->board_state;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get board state.");
            response->success = false;
            response->message = "Failed to get board state.";
            return;
        }

        // Create a request for /compute_best_move service
        auto compute_best_move_request = std::make_shared<moveit_planning::srv::ComputeBestMove::Request>();
        compute_best_move_request->board_state = board_state;
        // Send request
        auto compute_best_move_future = compute_best_move_client_->async_send_request(compute_best_move_request);
        // Handle response
        status = compute_best_move_future.wait_for(3s);
        int best_move, player;
        std::string game_status;
        if (status == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Received best move.");
            auto compute_best_move_response = compute_best_move_future.get();
            best_move = compute_best_move_response->best_move;
            player = compute_best_move_response->player;
            game_status = compute_best_move_response->game_status;
            RCLCPP_INFO(this->get_logger(), "Best move is: %d", best_move);
            RCLCPP_INFO(this->get_logger(), "Current player: %d", player);
            RCLCPP_INFO(this->get_logger(), "Game status: %s", game_status.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get best move.");
            response->success = false;
            response->message = "Failed to get best move.";
            return;
        }

        // Create a request for /move_to_coordinates service
        auto move_to_coordinates_request = std::make_shared<moveit_planning::srv::MoveToCoordinates::Request>();
        auto coordinates = this->coordinates_mapping_[best_move];
        move_to_coordinates_request->x = coordinates[0];
        move_to_coordinates_request->y = coordinates[1];
        move_to_coordinates_request->z = coordinates[2];
        move_to_coordinates_request->mode = player;
        RCLCPP_INFO(this->get_logger(), "Moving to coordinates X: %.3f, Y:%.3f, Z:%.3f.", coordinates[0], coordinates[1], coordinates[2]);
        // Send request
        auto move_to_coordinates_future = move_to_coordinates_client_->async_send_request(move_to_coordinates_request);
        // Handle response
        status = move_to_coordinates_future.wait_for(45s);
        bool service_success;
        if (status == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Received arm move response.");
            auto move_to_coordinates_response = move_to_coordinates_future.get();
            service_success = move_to_coordinates_response->success;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get arm move response.");
            response->success = false;
            response->message = "Failed to get arm move response.";
            return;
        }
        // Set response for main request
        if (service_success) {
            RCLCPP_INFO(this->get_logger(), "The arm was moved successfully.");
            response->success = true;
            response->message = game_status;
            return;
        } else {
            RCLCPP_ERROR(this->get_logger(), "The arm failed to perform move.");
            response->success = false;
            response->message = game_status;
        }
    }

    void start_new_game_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;  // to avoid unused parameter warning

        RCLCPP_INFO(this->get_logger(), "Received request to start a new game.");

        auto move_to_coordinates_request = std::make_shared<moveit_planning::srv::MoveToCoordinates::Request>();
        move_to_coordinates_request->x = 0.0;
        move_to_coordinates_request->y = 0.0;
        move_to_coordinates_request->z = 0.0;
        move_to_coordinates_request->mode = 3;  // Mode 3 for cleaning and drawing grid

        move_to_coordinates_client_->async_send_request(move_to_coordinates_request,
            [this, response](rclcpp::Client<moveit_planning::srv::MoveToCoordinates>::SharedFuture future) {
                auto move_to_coordinates_response = future.get();
                if (move_to_coordinates_response != nullptr && move_to_coordinates_response->success) {
                    RCLCPP_INFO(this->get_logger(), "New game started successfully.");
                    response->success = true;
                    response->message = "New game started successfully";
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to start new game.");
                    response->success = false;
                    response->message = "Failed to start new game";
                }
            }
        );
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TicTacToeOrchestrator>();
    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    // Add the node to the executor
    executor.add_node(node);
    executor.spin();
    // Shutdown when spinning is finished
    rclcpp::shutdown();
    return 0;
}