import random
import cv2
import time
import numpy as np
from math import pi
import os
import sys

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

from ClassicalAlgorithms.CVAlgorithm import CVAlgorithm
from TicTacToe.TicTacToeGame import *
CA = CVAlgorithm()

def localization_display(data_publisher, camera, last_results, is_debug=False):
    '''Localization'''
    top_left, bottom_right = last_results[0], last_results[1]
    min_u, min_v = top_left
    max_u, max_v = bottom_right
    while True:
        if is_debug:
            print('localizing...')
        color_image, _ = camera.getImage()
        gray = CA.color2gray(color_image)
        # o_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22,
        #                              maxRadius=25)
        pieces = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=33, maxRadius=38)
        uvr = []
        if pieces is not None:
            pieces = np.uint16(np.around(pieces))  # shape (1, n, 3)
            for i in range(pieces.shape[1]):
                u, v, r = pieces[0, i]
                if u<min_u or max_u<u or v<min_v or max_v<v:
                    uvr.append([u, v, r])
                    if is_debug:
                        cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)

            if is_debug:
                data_publisher.setData(color_image)
                # cv2.imshow('pieces', color_image)
                # cv2.waitKey(1000)
            return uvr

def identification_display(data_publisher, camera, last_results, is_debug=False):
    '''Identification'''
    uvr, top_left, bottom_right, piece_type = last_results[0], last_results[1], last_results[2], last_results[3]
    min_u, min_v = top_left
    max_u, max_v = bottom_right
    while True:
        if is_debug:
            print('identifying...')
        color_image, _ = camera.getImage()
        gray = CA.color2gray(color_image)
        o_pieces = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22, maxRadius=28)
        x_uvr, o_uvr = [], []
        if o_pieces is not None:
            o_pieces = np.uint16(np.around(o_pieces))  # shape (1, n, 3)
            for i in range(o_pieces.shape[1]):
                u, v, r = o_pieces[0, i]
                if u<min_u or max_u<u and v<min_v and max_v<v:
                    o_uvr.append([u, v, r])

            for i in range(len(uvr)):
                flag = True
                u, v, r = uvr[i][0], uvr[i][1], uvr[i][2]
                for j in range(len(o_uvr)):
                    uo, vo, ro = o_uvr[j][0], o_uvr[j][1], o_uvr[j][2]
                    # print((u-uo), (v-vo))
                    if (np.square(u-uo)+np.square(v-vo)) <= 100:
                        flag = False
                        break
                if flag:
                    x_uvr.append([u, v, r])
            if piece_type==0:
                if is_debug:
                    for xxx in x_uvr:
                        u, v, r = xxx[0], xxx[1], xxx[2]
                        cv2.circle(color_image, (u, v), r, (0, 0, 255), 2)
                    data_publisher.setData(color_image)
                    # cv2.imshow('red_pieces', color_image)
                    # cv2.waitKey(1000)
                return x_uvr
            else:
                if is_debug:
                    for xxx in o_uvr:
                        u, v, r = xxx[0], xxx[1], xxx[2]
                        cv2.circle(color_image, (u, v), r, (255, 0, 0), 2)
                    data_publisher.setData(color_image)
                    # cv2.imshow('blue_pieces', color_image)
                    # cv2.waitKey(1000)
                return o_uvr

def multiple_points_motion_planning(data_publisher, robot, last_results, is_debug=False):
    '''Multiple Points Motion Planning'''
    selected_pieces_uvr, game, board_pix, piece_type = last_results[0], last_results[1], last_results[2], last_results[3]

    pick_num = random.randint(0, len(selected_pieces_uvr)-1)
    pick_u, pick_v = selected_pieces_uvr[pick_num][0], selected_pieces_uvr[pick_num][1]

    if piece_type==0:
        ai_move = make_best_move(game, -1, "X")
        game.makeMove(ai_move, "X")
    else:
        ai_move = make_best_move(game, -1, "O")
        game.makeMove(ai_move, "O")
    place_u, place_v = board_pix[ai_move][0], board_pix[ai_move][1]

    pick_location = robot.calibration_tool.cvt(pick_u, pick_v)
    place_location = robot.calibration_tool.cvt(place_u, place_v)

    return pick_location, place_location


def execution_display(data_publisher, robot, end_effector, last_results, is_debug=False):
    '''Execution'''
    pick_location, place_location = last_results[0], last_results[1]

    robot.movej(pick_location[0], pick_location[1], robot.PICK_Z + 0.01, pi, 0, 0)
    robot.movej(pick_location[0], pick_location[1], robot.PICK_Z, pi, 0, 0)
    end_effector.closeGripper()
    # robot.goHome()

    robot.movej(place_location[0], place_location[1], robot.PLACE_Z + 0.01, pi, 0, 0)
    robot.movej(place_location[0], place_location[1], robot.PLACE_Z, pi, 0, 0)
    end_effector.openGripper()
    robot.goHome()

def board_detect(camera):
    centers = []
    while len(centers)!=4:
        print('board detecting...')
        color_background, _ = camera.getImage()
        contours = CA.find_contours(color_background, threshold=50)
        contours = CA.contours_area_filter(contours, 5800, 6800)
        centers = CA.find_contours_center(contours)
        # cv2.drawContours(c, contours, -1, (0, 255, 0), 1)
        cx, cy = 0, 0
        for center in centers:
            # cv2.circle(color_background, (center[0], center[1]), 1, (0, 255, 0), 2)
            cx+=center[0]
            cy+=center[1]
    tlu, tlv = cx/4-100, cy/4-100
    bru, brv = cx/4+100, cy/4+100
    delta_w, delta_h = int(float(bru - tlu) / 3), int(float(brv - tlv) / 3)
    board_pix = np.array([[tlu + delta_w / 2, tlv + delta_h / 2], [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2],
                          [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2],
                          [tlu + delta_w / 2, tlv + delta_h / 2 + delta_h],
                          [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + delta_h],
                          [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + delta_h],
                          [tlu + delta_w / 2, tlv + delta_h / 2 + 2 * delta_h],
                          [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + 2 * delta_h],
                          [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + 2 * delta_h]])
    for pix in board_pix:
        cv2.circle(color_background, (pix[0], pix[1]), 3, (0, 255, 255), -1)
    return color_background, board_pix, (tlu, tlv), (bru, brv)

# def pieces_detect(color_image, type):
#     if type == 'O':
#         gray = CA.color2gray(color_image)
#         circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22,
#                                    maxRadius=25)
#         uvr = []
#         if circles is not None:
#             circles = np.uint16(np.around(circles))  # shape (1, n, 3)
#             for i in range(circles.shape[1]):
#                 u, v, r = circles[0, i]
#                 uvr.append([u, v, r])
#                 cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)
#             return True, color_image, uvr
#         return False, color_image, uvr
#     elif type == 'X':
#         gray = CA.color2gray(color_image)
#         # gray = CA.color2binary(color_image, threshold=200, model=cv2.THRESH_BINARY_INV)
#         o_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22,
#                                    maxRadius=25)
#
#         circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=28,
#                                      maxRadius=35)
#         uvr = []
#         if circles is not None and o_circles is not None:
#             o_circles = np.uint16(np.around(o_circles))  # shape (1, n, 3)
#             circles = np.uint16(np.around(circles))  # shape (1, n, 3)
#             for i in range(circles.shape[1]):
#                 fff = True
#                 u, v, r = circles[0, i]
#                 for j in range(o_circles.shape[1]):
#                     uo, vo, ro = o_circles[0, j]
#                     print(uo, vo, ro, u, v, r)
#                     if (np.abs(u-uo)+np.abs(v-vo)) <= 20:
#                         fff = False
#                         break
#                 if fff:
#                     uvr.append([u, v, r])
#                     cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)
#
#             return True, color_image, uvr
#         return False, color_image, uvr
#     elif type=="ALL":
#         gray = CA.color2gray(color_image)
#         circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=12,
#                                    maxRadius=14)
#         uvr = []
#         if circles is not None:
#             circles = np.uint16(np.around(circles))  # shape (1, n, 3)
#             for i in range(circles.shape[1]):
#                 u, v, r = circles[0, i]
#                 uvr.append([u, v, r])
#                 cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)
#             return True, color_image, uvr
#         return False, color_image, uvr
#
# # def board_detect(color_background):
# #     # print(root_path+'/Data/TicTacToe/chessboard.png')
# #     template = cv2.imread(os.path.dirname(root_path)+'/Data/TicTacToe/chessboard.png', 0)
# #     _, top_left, bottom_right = CA.match_template(color_background[200:900, 100:800], template)
# #     tlu, tlv = top_left
# #     bru, brv = bottom_right
# #     delta_w, delta_h = int(float(bru - tlu) / 3), int(float(brv - tlv) / 3)
# #     board_pix = np.array([[tlu + delta_w / 2, tlv + delta_h / 2], [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2],
# #                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2],
# #                           [tlu + delta_w / 2, tlv + delta_h / 2 + delta_h],
# #                           [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + delta_h],
# #                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + delta_h],
# #                           [tlu + delta_w / 2, tlv + delta_h / 2 + 2 * delta_h],
# #                           [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + 2 * delta_h],
# #                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + 2 * delta_h]])
# #     for pix in board_pix:
# #         cv2.circle(color_background, (pix[0], pix[1]), 3, (0, 0, 255), -1)
# #     return color_background, board_pix, top_left, bottom_right
#
# def board_detect(camera):
#     centers = []
#     while len(centers)!=4:
#         color_background, depth_image, image_L, image_R = camera.getImage()
#         contours = CA.find_contours(color_background, threshold=50)
#         contours = CA.contours_area_filter(contours, 4000, 4500)
#         centers = CA.find_contours_center(contours)
#         # cv2.drawContours(c, contours, -1, (0, 255, 0), 1)
#         cx, cy = 0, 0
#         for center in centers:
#             # cv2.circle(color_background, (center[0], center[1]), 1, (0, 255, 0), 2)
#             cx+=center[0]
#             cy+=center[1]
#     tlu, tlv = cx/4-100, cy/4-100
#     bru, brv = cx/4+100, cy/4+100
#     delta_w, delta_h = int(float(bru - tlu) / 3), int(float(brv - tlv) / 3)
#     board_pix = np.array([[tlu + delta_w / 2, tlv + delta_h / 2], [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2],
#                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2],
#                           [tlu + delta_w / 2, tlv + delta_h / 2 + delta_h],
#                           [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + delta_h],
#                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + delta_h],
#                           [tlu + delta_w / 2, tlv + delta_h / 2 + 2 * delta_h],
#                           [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + 2 * delta_h],
#                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + 2 * delta_h]])
#     for pix in board_pix:
#         cv2.circle(color_background, (pix[0], pix[1]), 3, (0, 255, 255), -1)
#     return color_background, board_pix, (tlu, tlv), (bru, brv)
#
# def search_pieces(monitor, camera, board_region, type):
#     # Picking
#     top_left, bottom_right = board_region
#     min_u, min_v = top_left
#     max_u, max_v = bottom_right
#     flag = False
#     while flag == False:
#         print('searching available pieces...')
#         color_image, depth_image, image_L, image_R = camera.getImage()
#         f, color_image, uvr = pieces_detect(color_image, type)
#         # monitor.setData(color_image)
#         # cv2.imshow('pieces', color_image)
#         # cv2.waitKey(1)
#         if f:
#             for proposed_uv in uvr:
#                 u, v = proposed_uv[0], proposed_uv[1]
#                 if min_u <= u and u <= max_u and min_v <= v and v <= max_v:
#                     print(u, v)
#                 else:
#                     flag = True
#                     break
#     return u, v, 0.0
#
# def detect_person_move(color_image, background, game, checkerboard_pix):
#     # should be rewritten, return 'False/True, -1/0~8'
#     for i in range(9):
#         u, v = checkerboard_pix[i]
#         tem = sum(np.abs(
#             np.mean(background[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(400, 3), axis=0) - np.mean(
#                 color_image[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(20 * 20, 3), axis=0)) > 60)
#         print(np.abs(
#             np.mean(background[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(400, 3), axis=0) - np.mean(
#                 color_image[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(20 * 20, 3), axis=0)))
#         if tem > 0:
#             if game.board[i] == " ":
#                 person_move = i
#                 print("Detected person_move is at : %s" % person_move)
#                 return True, person_move
#     print("Fail to detect the person move!")
#     return False, -1
#
# def board_scan_once(color_image, background, checkerboard_pix):
#     flag, color_image, urv = pieces_detect(color_image, 'O')
#     # cv2.imshow('circle', color_image)
#     # cv2.waitKey(1)
#     judgment = []
#     for i in range(9):
#         u, v = checkerboard_pix[i]
#         ltem = sum(np.abs(np.mean(background[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(400, 3), axis=0) -
#                           np.mean(color_image[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(400, 3),
#                                   axis=0)) > 50)
#         if ltem > 0:
#             ltem = 4
#             for circle in urv:
#                 # print(np.abs(u-circle[0])+np.abs(v-circle[1]))
#                 if np.abs(u - circle[0]) + np.abs(v - circle[1]) < 20:
#                     ltem = 5
#                     break
#         judgment.append(ltem)
#     return judgment
#
# def board_scan(camera, background, board_pix):
#     circles_detect = np.zeros(9, dtype='int')
#     x_detect = np.zeros(9, dtype='int')
#     results = np.zeros(9, dtype='int')
#
#     for i in range(15):
#         color_image, depth_image, image_L, image_R = camera.getImage()
#         tem_judgment = board_scan_once(color_image, background, board_pix)
#         for j in range(9):
#             if tem_judgment[j] == 5:
#                 circles_detect[j] += 1
#             if tem_judgment[j] == 4:
#                 x_detect[j] += 1
#
#     results[circles_detect >= 7] = 5
#     results[x_detect >= 7] = 4
#     return results
#
# def update_board(game, judgment, ai_move, type):
#     if type=='O':
#         if judgment[ai_move] != 5:
#             print("updating board...")
#             game.board[ai_move] = " "
#             for i in range(9):
#                 if game.board[i] == " " and judgment[i] == 5:
#                     game.makeMove(i, "O")
#                     break
#             game.show()
#     elif type=='X':
#         if judgment[ai_move] != 4:
#             print("updating board...")
#             game.board[ai_move] = " "
#             for i in range(9):
#                 if game.board[i] == " " and judgment[i] == 4:
#                     game.makeMove(i, "X")
#                     break
#             game.show()
