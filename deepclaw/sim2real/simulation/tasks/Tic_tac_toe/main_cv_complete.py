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

def cv_get_position(img,depth_image):
    '''
    para
    ---
        img: np.ndarray type image
    return
    ---
        tuple of O chesses, X chesses and chessboard
        example: ([pos1, pos2, ...],[pos1, pos2, ...],pos)
    help
    ---
    area
        chess: ~=350
        chessboard: ~= 27700
    color
        X chess: blue HSV:[120 255 255]
        O chess: red HSV:[0 255 255]
        backgroud: yellow BRG:[0 255 255] HSV:[30 255 255] HSV_range:([20 200 200],[40 255 255])
    '''

    # create binay image
    upper_yellow = np.array([40, 255, 255])
    lower_yellow = np.array([20, 200, 200])

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    bin = cv2.inRange(hsv, lower_yellow, upper_yellow)
    bin = cv2.bitwise_not(bin)
    # find contours
    contours, _ = cv2.findContours(bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img, contours, -1, (0,255,0), 3)
    X_chesses = []
    O_chesses = []
    for cnt in contours:
        # filte by area
        area = cv2.contourArea(cnt)
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        if area < 400:# chess
            # filte by color
            color = img[cy,cx]
            hsv = cv2.cvtColor(np.array([[color]]), cv2.COLOR_BGR2HSV)
            if cv2.inRange(hsv, np.array([100,200,200]), np.array([140,255,255]))[0][0]==255:# blue X
                X_chesses.append((cam.H@cam.uv2XYZ(depth_image,cx,cy))[0:3])
            else: # red O
                O_chesses.append((cam.H@cam.uv2XYZ(depth_image,cx,cy))[0:3])
        elif 20000 < area and area < 30000: # chessboard
            chessboard = cam.H@cam.uv2XYZ(depth_image,cx,cy)
    return O_chesses, X_chesses, chessboard[0:3]

def move_chess_pipeline(chess_pos, chessboard_pos, action):
    # get the pos to put
    put_pos = np.array(chessboard_pos) + chessboard_bias_position(action)
    
    # get
    franka.clear_path = True
    chess_pos[2] += 0.1
    franka.move(env,chess_pos,euler=[0,np.radians(180),0])
    chess_pos[2] -= 0.11
    franka.move(env,chess_pos,euler=[0,np.radians(180),0])
    for chess in chesses:
        if franka.gripper._proximity_sensor.is_detected(chess):
            break
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
'''
img = cv2.imread('img.png')
env = Env(scene('Tic_tac_toe.ttt'))
env.start()
cam = Camera()

depth_image = np.load('depth_img.npy')
O_chesses, X_chesses, chessboard = cv_get_position(img,depth_image)
print(chessboard)

env.stop()
env.shutdown()
cv2.imshow('img',img)
cv2.waitKey()
'''
if __name__ == "__main__":
    env = Env(scene('Tic_tac_toe.ttt'))
    env.start()

    # franka
    franka = Franka()
    # set franka to home joints 
    franka.home(env)

    # initiate the camera and get an color image and depth image
    cam = Camera()
    img = cam.capture_bgr()
    depth_img = cam.capture_depth(in_meters=True)

    # TODO: detect the location of the chessboard and the chess pieces from the image
    O_chesses_pos, X_chesses_pos, chessboard_pos = cv_get_position(img,depth_img)
    
    # chessboard and chesses
    chesses_pos = {'X':X_chesses_pos,'O':O_chesses_pos}
    chesses = [Shape('chess_X'+str(chess_num)) for chess_num in range(1,6)]+\
              [Shape('chess_O'+str(chess_num)) for chess_num in range(1,6)]
    
    # create chessboard instance
    board = Board()
    
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
    
    # mian game loop
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
            chess = chesses_pos[player_chess][player_chess_count]
            player_chess_count += 1
            move_chess_pipeline(chess,chessboard_pos,action)
            turn = "AI"

        elif turn == "AI":
            # find the optimal move given the current board state using minimax
            action = ai.think(board)
            board._move(action,Ai_chess)

            # get AI chess and let the robot execute its move
            chess = chesses_pos[Ai_chess][Ai_chess_count]
            Ai_chess_count += 1
            move_chess_pipeline(chess,chessboard_pos,action)
            turn = 'player'
            
    input('Game over[Press Enter to quit]')
    env.stop()
    env.shutdown()