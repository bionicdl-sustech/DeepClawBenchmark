import os
import numpy as np
class Board(object):
    def __init__(self):
        self._board = ['-' for _ in range(9)]
        self._history = []

    def _move(self, action, take):
        if self._board[action] == '-':
            self._board[action] = take

            self._history.append((action, take))

    def _unmove(self, action):
        self._board[action] = '-'

        self._history.pop()

    def get_board_snapshot(self):
        return self._board[:]

    def get_legal_actions(self):
        actions = []
        for i in range(9):
            if self._board[i] == '-':
                actions.append(i)
        return actions

    def check_avalible_action(self):
        actions = self.get_legal_actions()
        return bool(len(actions))

    def is_legal_action(self, action):
        return self._board[action] == '-'

    def teminate(self):
        board = self._board
        lines = [board[0:3], board[3:6], board[6:9], board[0::3], board[1::3], board[2::3], board[0::4], board[2:7:2]]

        if ['X'] * 3 in lines or ['O'] * 3 in lines or '-' not in board:
            return True
        else:
            return False

    def get_winner(self):
        board = self._board
        lines = [board[0:3], board[3:6], board[6:9], board[0::3], board[1::3], board[2::3], board[0::4], board[2:7:2]]

        if ['X'] * 3 in lines:
            return 0
        elif ['O'] * 3 in lines:
            return 1
        else:
            return 2

    def print_history(self):
        print(self._history)

    def set_borad(self, m):
        m = np.array(m)
        m = m.flatten()
        board = []
        for num in m:
            if num == 0:
                board.append('-')
            elif num == 1:
                board.append('O')
            elif num == 2:
                board.append('X')
        self._board = board

    def print_board(self):
        for i in range(3):
            j = i*3
            line = self._board[j:j+3]
            print(' '*3 + '| ' + line[0] + ' ' +line[1] + ' ' +line[2] + ' |')

class tictactoe_ai():

    def __init__(self, take='X'):
        self.take = take

    def think(self, board):
        #print('\rThinking...')
        take = ['X', 'O'][self.take == 'X']
        player = tictactoe_ai(take)
        _, action = self.minimax(board, player)
        #print('\rFind solution!')
        return action

    def minimax(self, board, player, depth=0):
        if self.take == "O":
            bestVal = -10
        else:
            bestVal = 10

        if board.teminate():
            if board.get_winner() == 0:
                return -10 + depth, None
            elif board.get_winner() == 1:
                return 10 - depth, None
            elif board.get_winner() == 2:
                return 0, None

        for action in board.get_legal_actions():
            board._move(action, self.take)
            val, _ = player.minimax(board, self, depth + 1)
            board._unmove(action)

            if self.take == "O":
                if val > bestVal:
                    bestVal, bestAction = val, action
            else:
                if val < bestVal:
                    bestVal, bestAction = val, action

        return bestVal, bestAction

if __name__ == "__main__":
    # Create Board instance
    os.system('clear')
    board = Board()
    # start
    input('Press enter to start')
    # Decide who first (X always first)
    instream = input('Do you want to play first hand?[y/n]')
    if instream != 'n':
        ai = tictactoe_ai('O')
        player_chess = 'X'
        Ai_chess = 'O'
        turn = 'player'
    else:
        ai = tictactoe_ai('X')
        player_chess = 'O'
        Ai_chess = 'X'
        turn = 'AI'

    # mian game loop
    while True:
        os.system('clear')
        print(f'You = {player_chess}\tAi = {Ai_chess}\n')
        board.print_board()
        print('')
        res = board.get_winner()
        if res == 0:
            # X win
            winner = 'X'
            if player_chess == 'X':
                print('you win')
            else:
                print('you lose')
            break
        elif res == 1:
            # O win
            winner = 'O'
            if player_chess == 'O':
                print('you win')
            else:
                print('you lose')
            break
        elif res == 2:
            # check is avalible
            if not board.check_avalible_action():
                winner = 'no'
                print('no one win')
                break

        if turn == 'player':
            # print board
            # input a number to play chess
            # | 0 1 2 |
            # | 3 4 5 |
            # | 6 7 8 |
            instream = input('Play(input a integer):')
            action = int(instream)
            if not board.is_legal_action(action):
                continue
            board._move(action,player_chess)
            turn = "AI"

        elif turn == "AI":
            action = ai.think(board)
            board._move(int(action),Ai_chess)
            turn = 'player'
