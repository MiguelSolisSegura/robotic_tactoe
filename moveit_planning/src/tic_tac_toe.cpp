#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit_planning/srv/make_move.hpp"
#include "moveit_planning/srv/get_board_state.hpp"
#include "moveit_planning/srv/compute_best_move.hpp"
#include "moveit_planning/srv/move_to_coordinates.hpp"

using namespace std::chrono_literals;

class TicTacToeNode : public rclcpp::Node {
public:
    TicTacToeNode() : Node("tic_tac_toe_node") {
        this->declare_parameter<std::string>("config_file", "config.txt");
        game_done_ = false;

        service_ = this->create_service<moveit_planning::srv::MakeMove>(
            "make_move", std::bind(&TicTacToeNode::handle_make_move, this, std::placeholders::_1, std::placeholders::_2));
        
        client_get_board_state_ = this->create_client<moveit_planning::srv::GetBoardState>("get_board_state");
        client_compute_best_move_ = this->create_client<moveit_planning::srv::ComputeBestMove>("compute_best_move");
        client_move_to_coordinates_ = this->create_client<moveit_planning::srv::MoveToCoordinates>("move_to_coordinates");
    }

private:
    void handle_make_move(
        const std::shared_ptr<moveit_planning::srv::MakeMove::Request> request,
        std::shared_ptr<moveit_planning::srv::MakeMove::Response> response) {

        if (request->command == "New Game") {
            game_done_ = false;
            response->completed = true;
            return;
        }

        if (game_done_) {
            response->completed = false;
            RCLCPP_WARN(this->get_logger(), "Game is already finished. Start a new game to continue.");
            return;
        }

        if (request->command == "Next Move") {
            // Stage 1: Get board state
            auto board_state_response = call_get_board_state();
            if (!board_state_response) {
                response->completed = false;
                return;
            }

            // Stage 2: Compute best move
            auto best_move_response = call_compute_best_move(board_state_response->board_state);
            if (!best_move_response) {
                response->completed = false;
                return;
            }

            // Stage 3: Move to coordinates
            auto move_response = call_move_to_coordinates(best_move_response->best_move, best_move_response->player);
            if (!move_response || !move_response->success) {
                response->completed = false;
                return;
            }

            if (best_move_response->game_status != "Game in progress") {
                game_done_ = true;
            }

            response->completed = true;
            return;
        }

        response->completed = false;
    }

    std::shared_ptr<moveit_planning::srv::GetBoardState::Response> call_get_board_state() {
        auto request = std::make_shared<moveit_planning::srv::GetBoardState::Request>();
        while (!client_get_board_state_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_board_state service...");
        }

        auto future = client_get_board_state_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            return future.get();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call get_board_state service");
            return nullptr;
        }
    }

    std::shared_ptr<moveit_planning::srv::ComputeBestMove::Response> call_compute_best_move(const std::vector<int32_t>& board_state) {
        auto request = std::make_shared<moveit_planning::srv::ComputeBestMove::Request>();
        request->board_state = board_state;
        
        while (!client_compute_best_move_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for compute_best_move service...");
        }

        auto future = client_compute_best_move_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            return future.get();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call compute_best_move service");
            return nullptr;
        }
    }

    std::shared_ptr<moveit_planning::srv::MoveToCoordinates::Response> call_move_to_coordinates(int32_t best_move, int32_t player) {
        auto coordinates = get_coordinates_from_file(best_move);

        auto request = std::make_shared<moveit_planning::srv::MoveToCoordinates::Request>();
        request->x = coordinates[0];
        request->y = coordinates[1];
        request->z = coordinates[2];
        request->mode = player;

        while (!client_move_to_coordinates_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for move_to_coordinates service...");
        }

        auto future = client_move_to_coordinates_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            return future.get();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call move_to_coordinates service");
            return nullptr;
        }
    }

    std::vector<double> get_coordinates_from_file(int32_t move) {
        std::vector<double> coordinates(3, 0.0);
        std::string config_file;
        this->get_parameter("config_file", config_file);

        std::ifstream file(config_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open configuration file: %s", config_file.c_str());
            return coordinates;
        }

        int index;
        double x, y, z;
        while (file >> index >> x >> y >> z) {
            if (index == move) {
                coordinates[0] = x;
                coordinates[1] = y;
                coordinates[2] = z;
                break;
            }
        }

        file.close();
        return coordinates;
    }

    rclcpp::Service<moveit_planning::srv::MakeMove>::SharedPtr service_;
    rclcpp::Client<moveit_planning::srv::GetBoardState>::SharedPtr client_get_board_state_;
    rclcpp::Client<moveit_planning::srv::ComputeBestMove>::SharedPtr client_compute_best_move_;
    rclcpp::Client<moveit_planning::srv::MoveToCoordinates>::SharedPtr client_move_to_coordinates_;
    
    bool game_done_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TicTacToeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
