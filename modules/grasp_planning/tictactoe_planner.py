import os
import sys
import random
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.grasp_planning.grasp_planning import GraspPlaner


class TicTacToePlanner(GraspPlaner):
    def __init__(self, player, value=50, depth=3):
        self.player = player
        self.value = value
        self.depth = depth
        self.board = [" ", " ", " ",
                      " ", " ", " ",
                      " ", " ", " "]

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
        print("choices: ", choices)
        if len(choices) > 0:
            index = random.choice(choices)
        else:
            index = random.choice(self.available_moves())
        self.make_move(index, self.player)
        u, v = centers[index][0], centers[index][1]
        return [u, v, 0, 3.14, 0, 0]

    def show(self):
        """Format and print board"""
        print("""
          {} | {} | {}
         -----------
          {} | {} | {}
         -----------
          {} | {} | {}
        """.format(*self.board))

    def update_board(self, labels):
        # """
        # 4 | 3 | 2        0 | 1 | 2
        # ---------        ---------
        # 5 | 0 | 1  ===>  3 | 4 | 5
        # ---------        ---------
        # 6 | 7 | 8        6 | 7 | 8
        # """
        # new_labels = list(np.zeros(len(labels)))
        # new_labels[0], new_labels[1], new_labels[2] = labels[4], labels[3], labels[2]
        # new_labels[3], new_labels[4], new_labels[5] = labels[5], labels[0], labels[1]
        # new_labels[6], new_labels[7], new_labels[8] = labels[6], labels[7], labels[8]

        for i, label in enumerate(labels):
            if label == 1:
                self.board[i] = "B"
            elif label == 2:
                self.board[i] = "G"

    def change_player(self, player):
        if player == "G":
            return "B"
        else:
            return "G"

    def make_move(self, position, player):
        # print(player)
        self.board[position] = player
        # self.show()

    def available_moves(self):
        moves = []
        for i, player in enumerate(self.board):
            if player == " ":
                moves.append(i)
        return moves

    def get_moves(self, player):
        moves = []
        for i in range(0, len(self.board)):
            if self.board[i] == player:
                moves.append(i)
        return moves

    def check_win(self):
        combos = ([0, 1, 2], [3, 4, 5], [6, 7, 8],
                  [0, 3, 6], [1, 4, 7], [2, 5, 8],
                  [0, 4, 8], [2, 4, 6])

        for player in ("G", "B"):
            positions = self.get_moves(player)
            for combo in combos:
                win = True
                for pos in combo:
                    if pos not in positions:
                        win = False
                if win:
                    return player

    def game_over(self):
        if self.check_win() is not None:
            return True
        for player in self.board:
            if player == " ":
                return False
        return True

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
