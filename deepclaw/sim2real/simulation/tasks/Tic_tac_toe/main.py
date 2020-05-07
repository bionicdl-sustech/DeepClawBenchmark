'''
Tic Tac Toe
'''
from os.path import dirname, abspath
from os import system
sim_path = dirname(dirname(dirname(abspath(__file__))))
scene_path = sim_path + '/scene/'
import sys
sys.path.append(sim_path)
from src.camera import Camera
from src.env import Env
from src.franka import Franka
from pyrep.objects.shape import Shape
from Tic_tac_toe import Board, tictactoe_ai
import numpy as np
import cv2

def scene(scene_file_name):
    # return abs dir of scene file 
    return scene_path + scene_file_name

def chessboard_bias_position(action):
    a = 0.06*np.array([[[-1,-1,0], [-1,0,0], [-1,1,0]],
                        [[0,-1,0],  [0,0,0],  [0,1,0]],
                        [[1,-1,0],  [1,0,0],  [1,1,0]]]).reshape((-1,3))[action]
    return a
    
def move_chess_pipeline(chess_pos, chessboard_pos, action):
    # get the pos to put
    put_pos = np.array(chessboard_pos) + chessboard_bias_position(action)
    
    # get
    franka.clear_path = True
    chess_pos[2] += 0.1
    franka.move(env,chess_pos,euler=[0,np.radians(180),0])
    chess_pos[2] -= 0.1
    franka.move(env,chess_pos,euler=[0,np.radians(180),0])
    franka.grasp(env,chess)
    chess_pos[2] += 0.1
    franka.move(env,chess_pos,euler=[0,np.radians(180),0])

    # put
    put_pos[2] +=0.1
    franka.move(env,put_pos,euler=[0,np.radians(180),0])
    #put_pos[2] -=0.09
    #franka.move(env,put_pos,euler=[0,np.radians(180),0])
    franka.release(env)
    #put_pos[2] +=0.09
    #franka.move(env,put_pos,euler=[0,np.radians(180),0])
    franka.home(env)

if __name__ == "__main__":
    env = Env(scene('Tic_tac_toe.ttt'))
    env.start()

    # franka
    franka = Franka()
    # set franka to home joints 
    franka.home(env)

    # initiate the camera 
    cam = Camera()
    imgs = []
    
    # chessboard and chesses
    chessboard = Shape('chessboard')
    chesses = {'X':[Shape('chess_X'+str(chess_num)) for chess_num in range(1,6)],
                'O':[Shape('chess_O'+str(chess_num)) for chess_num in range(1,6)]}

    # create chessboard instance
    board = Board()
    imgs.append(cam.capture_bgr())

    # Decide who first (X always first)
    system('clear')
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
    
    # main game loop
    Ai_chess_count = 0
    player_chess_count = 0
    while True:
        system('clear')
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

            # check validity
            if not board.is_legal_action(action):
                continue
            '''
            if (action not in range(9)) and (not board.is_legal_action(action)):
                continue
            '''
            # move chess to chess board
            board._move(action,player_chess)

            # get the player chess and ask the robot to execute the player's move
            chess = chesses[player_chess][player_chess_count]
            player_chess_count += 1
            move_chess_pipeline(chess.get_position(),chessboard.get_position(),action)
            turn = "AI"

        elif turn == "AI":
            # find the optimal move given the current board state using minimax
            action = ai.think(board)
            board._move(action,Ai_chess)
            
            # get AI chess and let the robot execute its move, the position of the chess is read from the environment
            chess = chesses[Ai_chess][Ai_chess_count]
            Ai_chess_count += 1
            move_chess_pipeline(chess.get_position(),chessboard.get_position(),action)
            turn = 'player'

    input('Game over[Press Enter to quit]')
    env.stop()
    env.shutdown()
