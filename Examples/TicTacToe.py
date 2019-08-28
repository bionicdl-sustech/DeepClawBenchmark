import os
import sys

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)
# print(root_path)

from Functions.TicTacToe.TicTacToeGame import *
from Functions.TicTacToe.Functions import *
from ToolKit.DataCollector import ImagePublisher, TimePublisher, Monitor

image_publisher = ImagePublisher('image_pub')
time_publisher = TimePublisher('time_pub')
data_collector = Monitor('Tic-tac-toe-task5')
image_publisher.registerObserver(data_collector)
time_publisher.registerObserver(data_collector)

def subtask_display(perception_system, manipulation_system, args, is_debug=False):
    # args initial
    camera = perception_system[0]
    robot_arm,  robot_gripper = manipulation_system[0], manipulation_system[1]
    game, board_pix, top_left, bottom_right, ite = args[0], args[1], args[2], args[3], args[4]
    piece_type = ite % 2

    # localization step
    start_time = time.time()
    pieces_uvr = localization_display(image_publisher, camera, [top_left, bottom_right], is_debug)
    end_time = time.time()
    if is_debug:
        time_publisher.setData([str(ite)+'localization', end_time - start_time])

    # identification step
    start_time = time.time()
    selected_pieces_uvr = identification_display(image_publisher, camera,
                                                 [pieces_uvr, top_left, bottom_right, piece_type], is_debug)
    end_time = time.time()
    if is_debug:
        time_publisher.setData([str(ite)+'identification', end_time - start_time])

    # multiple points motion planning step
    start_time = time.time()
    pick_location, place_location = multiple_points_motion_planning(image_publisher, robot_arm,
                                                                    [selected_pieces_uvr, game, board_pix, piece_type])
    end_time = time.time()
    if is_debug:
        time_publisher.setData([str(ite)+'multiple_points_motion_planning', end_time - start_time])

    # execution step
    start_time = time.time()
    execution_display(image_publisher, robot_arm, robot_gripper, [pick_location, place_location])
    end_time = time.time()
    if is_debug:
        time_publisher.setData([str(ite)+'execution', end_time - start_time])

def task_display(perception_system, manipulation_system, is_debug=False):
    # game initial
    game = TicTacToe()

    # robot system
    robot_arm = manipulation_system['Arm']
    robot_gripper = manipulation_system['End-effector']

    # sensors
    camera = perception_system['Camera']

    # board detection
    color_image, board_pix, top_left, bottom_right = board_detect(camera)
    for i in range(20):
        color_image, board_pix, top_left, bottom_right = board_detect(camera)

    # print('ttt')
    if is_debug:
        # cv2.imshow('board', color_image)
        # cv2.waitKey(1000)
        image_publisher.setData(color_image)

    ite = 0
    while game.gameOver() == False:
        subtask_display([camera], [robot_arm, robot_gripper], [game, board_pix, top_left, bottom_right, ite], is_debug)
        ite += 1
        grasp_label = raw_input('grasp label[0/1/2]:')
        if is_debug:
            time_publisher.setData([str(i) + 'grasp label', str(grasp_label)])

