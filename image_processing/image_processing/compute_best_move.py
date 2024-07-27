#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_planning.srv import ComputeBestMove
import random

class BestMoveCalculator(Node):
    def __init__(self):
        super().__init__('best_move_calculator')
        self.srv = self.create_service(ComputeBestMove, 'compute_best_move', self.compute_best_move_callback)
        self.get_logger().info('Best move server started.')

    def compute_best_move_callback(self, request, response):
        self.get_logger().info('Received request to compute the best next move.')
        board = request.board_state
        best_move, player = self.find_best_move(board)
        game_status = self.evaluate_game_status(board, best_move, player)
        response.best_move = best_move
        response.player = player
        response.game_status = game_status
        self.get_logger().info(f'The best next move is {best_move} for player {player}.')
        self.get_logger().info(f'Current game status is: {game_status}.')
        return response

    def is_moves_left(self, board):
        return any(cell == 0 for cell in board)

    def evaluate(self, board):
        # Checking for Rows for X or O victory.
        for row in range(3):
            if board[row * 3] == board[row * 3 + 1] == board[row * 3 + 2]:
                if board[row * 3] == 1:
                    return 10
                elif board[row * 3] == 2:
                    return -10

        # Checking for Columns for X or O victory.
        for col in range(3):
            if board[col] == board[3 + col] == board[6 + col]:
                if board[col] == 1:
                    return 10
                elif board[col] == 2:
                    return -10

        # Checking for Diagonals for X or O victory.
        if board[0] == board[4] == board[8]:
            if board[0] == 1:
                return 10
            elif board[0] == 2:
                return -10

        if board[2] == board[4] == board[6]:
            if board[2] == 1:
                return 10
            elif board[2] == 2:
                return -10

        return 0

    def minimax(self, board, depth, is_max):
        score = self.evaluate(board)

        # If Maximizer has won the game, return evaluated score
        if score == 10:
            return score

        # If Minimizer has won the game, return evaluated score
        if score == -10:
            return score

        # If there are no more moves and no winner then it is a tie
        if not self.is_moves_left(board):
            return 0

        # If this maximizer's move
        if is_max:
            best = -1000

            # Traverse all cells
            for i in range(9):
                # Check if cell is empty
                if board[i] == 0:
                    # Make the move
                    board[i] = 1

                    # Call minimax recursively and choose the maximum value
                    best = max(best, self.minimax(board, depth + 1, not is_max))

                    # Undo the move
                    board[i] = 0
            return best

        # If this minimizer's move
        else:
            best = 1000

            # Traverse all cells
            for i in range(9):
                # Check if cell is empty
                if board[i] == 0:
                    # Make the move
                    board[i] = 2

                    # Call minimax recursively and choose the minimum value
                    best = min(best, self.minimax(board, depth + 1, not is_max))

                    # Undo the move
                    board[i] = 0
            return best

    def find_best_move(self, board):
        # Determine current player (1 or 2) based on the number of moves made
        num_moves = sum(1 for cell in board if cell != 0)
        current_player = 1 if num_moves % 2 == 0 else 2

        # If the board is completely empty, return a random move
        if num_moves == 0:
            return random.choice([i for i in range(9)]), current_player

        # Initialize best value and move
        best_val = -1000 if current_player == 1 else 1000
        best_move = -1

        # Traverse all cells, evaluate minimax function for all empty cells.
        for i in range(9):
            # Check if cell is empty
            if board[i] == 0:
                # Make the move
                board[i] = current_player

                # Compute evaluation function for this move.
                move_val = self.minimax(board, 0, current_player == 2)

                # Undo the move
                board[i] = 0

                # If the value of the current move is better, update best
                if (current_player == 1 and move_val > best_val) or (current_player == 2 and move_val < best_val):
                    best_move = i
                    best_val = move_val

        return best_move, current_player

    def evaluate_game_status(self, board, best_move, player):
        # Make the best move to evaluate the game status
        # board[best_move] = player
        score = self.evaluate(board)

        if score == 10:
            return "Player 1 wins"
        elif score == -10:
            return "Player 2 wins"
        elif not self.is_moves_left(board):
            return "There is a draw"
        else:
            return "Game in progress"

def main(args=None):
    rclpy.init(args=args)
    node = BestMoveCalculator()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
