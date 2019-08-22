import random
import cv2
import time
import numpy as np
from math import pi
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
data_collector = Monitor('obs1')
image_publisher.registerObserver(data_collector)
time_publisher.registerObserver(data_collector)

# def display(robot, end_effector, camera):
#     game = TicTacToe()
#
#     for i in range(20):
#         color_image, depth_image, image_L, image_R = camera.getImage()
#         background, board_pix, top_left, bottom_right = board_detect(color_image)
#     # cv2.imshow('background', background)
#     # cv2.waitKey(2000)
#
#     while game.gameOver() == False:
#         time.sleep(3)
#         detected = False
#         while not detected:
#             color_image, depth_image, image_L, image_R = camera.getImage()
#             detected, person_move = detect_person_move(color_image, background, game, board_pix)
#             if not detected:
#                 print("Please place you piece X again!")
#                 time.sleep(6)
#
#         game.makeMove(person_move, "X")
#         game.show()
#
#         if game.gameOver() == True:
#             break
#
#         print("Computer choosing move...")
#         ai_move = make_best_move(game, -1, "O")
#         game.makeMove(ai_move, "O")
#         game.show()
#         judgment = check_robot_play(robot, end_effector, camera, background,
#                                     board_pix, top_left, bottom_right, ai_move, type="X")
#         # game.show()
#         is_not_diff = True
#         while is_not_diff:
#             pre_judgment = board_scan(camera, background, board_pix)
#             raw_input('waiting for robot...')
#
#             pick_u, pick_v, pick_depth = search_pieces(camera, (top_left, bottom_right))
#             place_u, place_v, place_depth = board_pix[ai_move][0], board_pix[ai_move][1], 0.95
#             robot_display(robot, camera, [pick_u, pick_v, pick_depth], [place_u, place_v, place_depth])
#             # raw_input('wait')
#
#             post_judgment = board_scan(camera, background, board_pix)
#             print(pre_judgment, post_judgment)
#             for i in range(9):
#                 if pre_judgment[i] != post_judgment[i]:
#                     is_not_diff = False
#                     break
#
#         if post_judgment[ai_move] != 5:
#             print("updating board...")
#             game.board[ai_move] = " "
#             for i in range(9):
#                 if game.board[i] == " " and post_judgment[i] == 5:
#                     game.makeMove(i, "O")
#                     break
#             game.show()
#
#     print("Game Over. " + game.whoWon() + " Wins")

def robot_display(robot, end_effector, pick_location, place_location):
    # Picking
    location = robot.calibration_tool.cvt(pick_location[0], pick_location[1])
    robot.movej(location[0], location[1], robot.PICK_Z+0.01, pi, 0, 0)
    robot.movej(location[0], location[1], robot.PICK_Z, pi, 0, 0)
    # end_effector.closeGripper()
    robot.goHome()

    # Placing
    location = robot.calibration_tool.cvt(place_location[0], place_location[1])
    robot.movej(location[0], location[1], robot.PLACE_Z+0.01, pi, 0, 0)
    robot.movej(location[0], location[1], robot.PLACE_Z, pi, 0, 0)
    # end_effector.openGripper()
    robot.goHome()

# def check_robot_play(robot, end_effector, camera, background,
#                      board_pix, top_left, bottom_right, ai_move, type):
#     is_not_diff = True
#
#     pick_u, pick_v, pick_depth = search_pieces(camera, (top_left, bottom_right), type)
#     place_u, place_v, place_depth = board_pix[ai_move][0], board_pix[ai_move][1], 0.95
#     robot_display(robot, end_effector, [pick_u, pick_v, pick_depth], [place_u, place_v, place_depth])
#     return []
#
#     while is_not_diff:
#         pre_judgment = board_scan(camera, background, board_pix)
#
#         pick_u, pick_v, pick_depth = search_pieces(camera, (top_left, bottom_right), type)
#         place_u, place_v, place_depth = board_pix[ai_move][0], board_pix[ai_move][1], 0.95
#         robot_display(robot, end_effector, [pick_u, pick_v, pick_depth], [place_u, place_v, place_depth])
#
#         post_judgment = board_scan(camera, background, board_pix)
#         # print(pre_judgment, post_judgment)
#         for i in range(9):
#             if pre_judgment[i] != post_judgment[i]:
#                 is_not_diff = False
#                 break
#     return post_judgment

def auto_display(robot, end_effector, camera):
    game = TicTacToe()
    background = ''
    board_pix = ''
    top_left = ''
    bottom_right = ''

    for i in range(20):
        background, board_pix, top_left, bottom_right = board_detect(camera)

    cv2.imshow('board', background)
    cv2.waitKey(2000)
    # image_publisher.setData(background)

    while game.gameOver() == False:
        time.sleep(1)
        print("Computer X choosing move...")
        ai_move = make_best_move(game, -1, "X")

        game.makeMove(ai_move, "X")
        game.show()
        # judgment = check_robot_play(robot, end_effector, camera, background,
        #                             board_pix, top_left, bottom_right, ai_move, type="X")
        # update_board(game, judgment, ai_move, 'X')

        st = time.time()
        pick_u, pick_v, pick_depth = search_pieces(image_publisher, camera, (top_left, bottom_right), 'X')
        et = time.time()
        time_publisher.setData(['location + identification',et-st]) # time record

        place_u, place_v, place_depth = board_pix[ai_move][0], board_pix[ai_move][1], 0.95
        st = time.time()
        robot_display(robot, end_effector, [pick_u, pick_v, pick_depth], [place_u, place_v, place_depth])
        et = time.time()
        time_publisher.setData(['multiple points motion planning + execution', et - st])  # time record

        if game.gameOver() == True:
            break

        print("Computer O choosing move...")
        ai_move = make_best_move(game, -1, "O")
        game.makeMove(ai_move, "O")
        game.show()
        # judgment = check_robot_play(robot, end_effector, camera, background,
        #                  board_pix, top_left, bottom_right, ai_move, type="O")
        # update_board(game, judgment, ai_move, 'O')

        pick_u, pick_v, pick_depth = search_pieces(image_publisher, camera, (top_left, bottom_right), 'O')
        place_u, place_v, place_depth = board_pix[ai_move][0], board_pix[ai_move][1], 0.95
        st = time.time()
        robot_display(robot, end_effector, [pick_u, pick_v, pick_depth], [place_u, place_v, place_depth])
        et = time.time()
        time_publisher.setData(['multiple points motion planning + execution', et - st])  # time record

    print("Game Over. " + game.whoWon() + " Wins")

