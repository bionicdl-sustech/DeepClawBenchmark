import os
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.grasp_planning.grasp_planning import GraspPlaner


class TicTacToePlanner(GraspPlaner):
    def __init__(self, player):
        self.player = player
        self.board = [" ", " ", " ",
                      " ", " ", " ",
                      " ", " ", " "]

    def display(self, centers, labels, **kwargs):
        raise NotImplementedError("display not implement")

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
