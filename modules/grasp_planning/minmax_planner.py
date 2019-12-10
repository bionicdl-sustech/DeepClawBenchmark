import os
import sys
import random

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.grasp_planning.tictactoe_planner import TicTacToePlanner


class MinMaxPlanner(TicTacToePlanner):
    def __init__(self, player, value=50, depth=3):
        super(MinMaxPlanner, self).__init__(player)
        self.value = value
        self.depth = depth

    def display(self, centers, labels, **kwargs):
        self.update_board(labels)
        choices = []
        candidate_moves = self.available_moves()
        # print(candidate_moves)
        for move in candidate_moves:
            self.make_move(move, self.player)
            move_value = self.minimax(self.depth-1, self.change_player(self.player))
            self.make_move(move, " ")

            if self.player == "B":
                if move_value > self.value:
                    choices = [move]
                    break
                elif move_value == self.value:
                    choices.append(move)
            elif self.player == "G":
                if move_value < self.value:
                    choices = [move]
                    break
                elif move_value == self.value:
                    choices.append(move)
        # print("choices: ", choices)
        if len(choices) > 0:
            index = random.choice(choices)
        else:
            index = random.choice(self.available_moves())
        self.make_move(index, self.player)
        u, v = centers[index][0], centers[index][1]
        return [u, v, 0, 3.14, 0, 0]

    def minimax(self, depth, player):
        if depth == 0 or self.game_over():
            if self.check_win() == "G":
                return 0
            elif self.check_win() == "B":
                return 100
            else:
                return 50

        if player == "B":
            best_value = 0
            for move in self.available_moves():
                self.make_move(move, player)
                move_value = self.minimax(depth-1, self.change_player(player))
                self.make_move(move, " ")
                best_value = max(best_value, move_value)
            return best_value

        if player == "G":
            best_value = 100
            for move in self.available_moves():
                self.make_move(move, player)
                move_value = self.minimax(depth-1, self.change_player(player))
                self.make_move(move, " ")
                best_value = min(best_value, move_value)
            return best_value
