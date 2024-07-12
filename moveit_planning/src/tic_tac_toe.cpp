#include "rclcpp/rclcpp.hpp"
#include "moveit_planning/srv/get_board_state.hpp"
#include "moveit_planning/srv/compute_best_move.hpp"
#include "moveit_planning/srv/move_to_coordinates.hpp"
#include "moveit_planning/srv/make_move.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <cstdlib>
#include <iostream>

using namespace std::chrono_literals;
using namespace std::placeholders;

/**
 * @brief The TicTacToeOrchestrator class orchestrates the game of Tic-Tac-Toe
 * by interfacing with various ROS2 services to get the board state, compute
 * the best move, and move to the specified coordinates on the board.
 */
class TicTacToeOrchestrator : public rclcpp::Node {
public:
    /**
     * @brief Construct a new TicTacToeOrchestrator object
     */
    TicTacToeOrchestrator() : Node("tictactoe_orchestrator") {
        // Create a callback group
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Initialize clients
        get_board_state_client_ = this->create_client<moveit_planning::srv::GetBoardState>(
            "get_board_state", rmw_qos_profile_services_default, callback_group_);
        compute_best_move_client_ = this->create_client<moveit_planning::srv::ComputeBestMove>(
            "compute_best_move", rmw_qos_profile_services_default, callback_group_);
        move_to_coordinates_client_ = this->create_client<moveit_planning::srv::MoveToCoordinates>(
            "move_to_coordinates", rmw_qos_profile_services_default, callback_group_);
        
        // Initialize services
        ask_for_next_move_service_ = this->create_service<moveit_planning::srv::MakeMove>(
            "ask_for_next_move", std::bind(&TicTacToeOrchestrator::ask_for_next_move_callback, this, _1, _2),
            rmw_qos_profile_services_default, callback_group_);
        start_new_game_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_new_game", std::bind(&TicTacToeOrchestrator::start_new_game_callback, this, _1, _2));

        // Parameter for switching between simulation and real modes
        this->declare_parameter<bool>("sim", false);

        // Load mapping from configuration file
        load_coordinates_mapping();

        RCLCPP_INFO(this->get_logger(), "Main node tic-tac-toe orchestrator started.");
    }

private:
    // Clients
    rclcpp::Client<moveit_planning::srv::GetBoardState>::SharedPtr get_board_state_client_;
    rclcpp::Client<moveit_planning::srv::ComputeBestMove>::SharedPtr compute_best_move_client_;
    rclcpp::Client<moveit_planning::srv::MoveToCoordinates>::SharedPtr move_to_coordinates_client_;

    // Services
    rclcpp::Service<moveit_planning::srv::MakeMove>::SharedPtr ask_for_next_move_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_new_game_service_;

    // Callback group
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // Coordinates mapping
    std::map<int, std::array<float, 3>> coordinates_mapping_;

    // Symbols mapping
    std::map<int, std::string> symbols_mapping_ = {{1, "cross"}, {2, "circle"}};

    // Package location
    std::string package_directory_ = ament_index_cpp::get_package_share_directory("moveit_planning");
    std::string model_path_ = package_directory_ + "/models/"; 

    // Flag to control game flow
    bool terminal_state_ = false;

    /**
     * @brief Load the coordinates mapping from the sim_config.txt file.
     */
    void load_coordinates_mapping() {
        // Filename with coordinates
        bool sim = this->get_parameter("sim").as_bool();
        std::string file_name;
        if (sim) {
            file_name = "sim_config.txt";
        } else {
            file_name = "real_config.txt";
        }
        
        std::string config_file = this->package_directory_ + "/config/" + file_name; 

        std::ifstream file(config_file);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", file_name.c_str());
            return;
        }

        int move;
        float x, y, z;
        while (file >> move >> x >> y >> z) {
            coordinates_mapping_[move] = {x, y, z};
        }
        RCLCPP_INFO(this->get_logger(), "Loaded coordinates mapping from %s", file_name.c_str());
    }

    /**
     * @brief Callback function for the "ask_for_next_move" service.
     * 
     * @param request The service request.
     * @param response The service response.
     */
    void ask_for_next_move_callback(
        const std::shared_ptr<moveit_planning::srv::MakeMove::Request> request,
        std::shared_ptr<moveit_planning::srv::MakeMove::Response> response) {

        if (request->manual) {
            RCLCPP_INFO(this->get_logger(), "Received request to perform a manual move.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Received request to perform an intelligent move.");
        }
        
        if (terminal_state_) {
            RCLCPP_ERROR(this->get_logger(), "The game is in a terminal state. Please start a new game.");
            response->success = false;
            response->message = "The game is in a terminal state. Please start a new game.";
            return;
        }

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

        if (request->manual) {best_move = request->command;}

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
            // Set flag if the game is in a terminal state
            if (game_status != "Game in progress") {terminal_state_ = true;}
        } else {
            RCLCPP_ERROR(this->get_logger(), "The arm failed to perform move.");
            response->success = false;
            response->message = game_status;
            return;
        }

        bool sim = this->get_parameter("sim").as_bool();

        if (sim) {
            // Define the command to spawn the entity
            std::string spawn_command = "ros2 run gazebo_ros spawn_entity.py -entity " + symbols_mapping_[player] + "_" + 
                std::to_string(best_move) + " -file " + this->model_path_ + symbols_mapping_[player] + "/model.sdf -x " +
                std::to_string(coordinates[0] + 1.0) + " -y " + std::to_string(coordinates[1]) + " -z 0.02";
            // Execute the command
            system(spawn_command.c_str());
        }
    }

    /**
     * @brief Callback function for the "start_new_game" service.
     * 
     * @param request The service request.
     * @param response The service response.
     */
    void start_new_game_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        (void)request;  // Avoid unused parameter warning

        RCLCPP_INFO(this->get_logger(), "Received request to start a new game.");

    	// Create request object
        auto move_to_coordinates_request = std::make_shared<moveit_planning::srv::MoveToCoordinates::Request>();
        // Clean the board
        move_to_coordinates_request->mode = 5;
        // Send request
        auto move_to_coordinates_future = move_to_coordinates_client_->async_send_request(move_to_coordinates_request);
        // Handle response
        auto status = move_to_coordinates_future.wait_for(30s);
        bool service_success;
        if (status == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Received arm move response for clear request.");
            auto move_to_coordinates_response = move_to_coordinates_future.get();
            service_success = move_to_coordinates_response->success;
            if (!service_success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to clear the board.");
                response->success = false;
                response->message = "Failed to clear the board.";
                return;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Timeout: Failed to clear the board.");
            response->success = false;
            response->message = "Timeout: Failed to clear the board.";
            return;
        }
        
        // Draw the grid
        std::array<float, 3> coordinates = this->coordinates_mapping_[4];
        move_to_coordinates_request->x = coordinates[0];
        move_to_coordinates_request->y = coordinates[1];
        move_to_coordinates_request->z = coordinates[2];
        move_to_coordinates_request->mode = 3;
        // Send request
        move_to_coordinates_future = move_to_coordinates_client_->async_send_request(move_to_coordinates_request);
        // Handle response
        status = move_to_coordinates_future.wait_for(45s);
        if (status == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Received arm move response for grid drawing request.");
            auto move_to_coordinates_response = move_to_coordinates_future.get();
            service_success = move_to_coordinates_response->success;
            if (!service_success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to draw the grid.");
                response->success = false;
                response->message = "Failed to draw the grid.";
                return;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Timeout: Failed to draw the grid.");
            response->success = false;
            response->message = "Timeout: Failed to draw the grid.";
            return;
        }

        // Set flag for terminal state
        terminal_state_ = false;

        bool sim = this->get_parameter("sim").as_bool();

        if (sim) {
            // Define the command to spawn the entity
            std::string spawn_command = "ros2 run gazebo_ros spawn_entity.py -entity grid -file " +
                                        this->model_path_ + "grid/model.sdf -x 1.2 -y 0.02 -z 0.02";
            // Execute the command
            system(spawn_command.c_str());
        }
        
        response->success = service_success;
        response->message = "The board was cleared and a new grid is available.";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TicTacToeOrchestrator>();
    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    // Add the node to the executor
    executor.add_node(node);
    // Spin the executor
    executor.spin();
    // Shutdown when spinning is finished
    rclcpp::shutdown();
    return 0;
}
