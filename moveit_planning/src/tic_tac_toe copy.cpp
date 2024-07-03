#include "rclcpp/rclcpp.hpp"
#include "moveit_planning/srv/get_board_state.hpp"
#include "moveit_planning/srv/compute_best_move.hpp"
#include "moveit_planning/srv/move_to_coordinates.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

class TicTacToeOrchestrator : public rclcpp::Node {
public:
    TicTacToeOrchestrator() : Node("tictactoe_orchestrator") {
        // Initialize clients
        get_board_state_client_ = this->create_client<moveit_planning::srv::GetBoardState>("get_board_state");
        compute_best_move_client_ = this->create_client<moveit_planning::srv::ComputeBestMove>("compute_best_move");
        move_to_coordinates_client_ = this->create_client<moveit_planning::srv::MoveToCoordinates>("move_to_coordinates");

        // Initialize services
        ask_for_next_move_service_ = this->create_service<std_srvs::srv::Trigger>("ask_for_next_move", std::bind(&TicTacToeOrchestrator::ask_for_next_move_callback, this, std::placeholders::_1, std::placeholders::_2));
        start_new_game_service_ = this->create_service<std_srvs::srv::Trigger>("start_new_game", std::bind(&TicTacToeOrchestrator::start_new_game_callback, this, std::placeholders::_1, std::placeholders::_2));

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
        (void)request;  // to avoid unused parameter warning

        RCLCPP_INFO(this->get_logger(), "Received request to determine and perform the next move.");

        auto get_board_state_request = std::make_shared<moveit_planning::srv::GetBoardState::Request>();
        get_board_state_client_->async_send_request(get_board_state_request,
            [this, response](rclcpp::Client<moveit_planning::srv::GetBoardState>::SharedFuture future) {
                auto board_state_response = future.get();
                if (board_state_response == nullptr) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get board state.");
                    response->success = false;
                    response->message = "Failed to get board state";
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Received board state.");
                this->handle_board_state_response(board_state_response->board_state, response);
            }
        );
    }

    void handle_board_state_response(const std::array<int32_t, 9>& board_state, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Handling board state response.");
        auto compute_best_move_request = std::make_shared<moveit_planning::srv::ComputeBestMove::Request>();
        compute_best_move_request->board_state = board_state;

        compute_best_move_client_->async_send_request(compute_best_move_request,
            [this, response, board_state](rclcpp::Client<moveit_planning::srv::ComputeBestMove>::SharedFuture future) {
                auto best_move_response = future.get();
                if (best_move_response == nullptr) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to compute best move.");
                    response->success = false;
                    response->message = "Failed to compute best move";
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Best move: %d, Player: %d, Game status: %s",
                            best_move_response->best_move, best_move_response->player, best_move_response->game_status.c_str());

                if (best_move_response->game_status != "Game in progress") {
                    response->success = true;
                    response->message = best_move_response->game_status;
                    return;
                }

                auto coordinates = this->coordinates_mapping_[best_move_response->best_move];
                RCLCPP_INFO(this->get_logger(), "Coordinates for move: x=%f, y=%f, z=%f",
                            coordinates[0], coordinates[1], coordinates[2]);
                this->handle_move_to_coordinates(coordinates, best_move_response->player, response);
            }
        );
    }

    void handle_move_to_coordinates(const std::array<double, 3>& coordinates, int player, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Moving to coordinates for player %d.", player);
        auto move_to_coordinates_request = std::make_shared<moveit_planning::srv::MoveToCoordinates::Request>();
        move_to_coordinates_request->x = coordinates[0];
        move_to_coordinates_request->y = coordinates[1];
        move_to_coordinates_request->z = coordinates[2];
        move_to_coordinates_request->mode = player;

        move_to_coordinates_client_->async_send_request(move_to_coordinates_request,
            [this, response](rclcpp::Client<moveit_planning::srv::MoveToCoordinates>::SharedFuture future) {
                auto move_to_coordinates_response = future.get();
                if (move_to_coordinates_response != nullptr && move_to_coordinates_response->success) {
                    RCLCPP_INFO(this->get_logger(), "Move performed successfully.");
                    response->success = true;
                    response->message = "Move performed successfully";
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to perform move.");
                    response->success = false;
                    response->message = "Failed to perform move";
                }
            }
        );
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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}